#include "odrive_manager.hpp"
#include "ODriveFlexCAN.hpp"
#include <Arduino.h>

Motor::Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int node_id)
    : odrive(wrap_can_intf(can_intf), node_id),
      controller(0.0, 0.0, 0.0),
      id(node_id) {}

namespace {
// FlexCAN_T4's onReceive callback is a plain function pointer with no user
// data. We route it through this file-scope pointer set in setup_can().
ODriveManager* g_instance = nullptr;

void can_message_trampoline(const CanMsg& msg) {
    if (g_instance) g_instance->on_can_message(msg);
}
}  // namespace

ODriveManager::ODriveManager()
    : motors_{Motor(can_intf_, 0), Motor(can_intf_, 1), Motor(can_intf_, 2)} {}

void ODriveManager::on_heartbeat(Heartbeat_msg_t& msg, void* user_data) {
    auto* ud = static_cast<ODriveUserData*>(user_data);
    ud->last_heartbeat = msg;
    ud->received_heartbeat = true;
}

void ODriveManager::on_position_feedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
    auto* ud = static_cast<ODriveUserData*>(user_data);
    ud->last_position = msg;
    ud->received_position = true;
}

void ODriveManager::on_torque_feedback(Get_Torques_msg_t& msg, void* user_data) {
    auto* ud = static_cast<ODriveUserData*>(user_data);
    ud->last_torque = msg;
    ud->received_torque = true;
}

void ODriveManager::on_can_message(const CanMsg& msg) {
    // Standard ODrive CAN ID layout: upper bits are node ID.
    uint8_t node_id = msg.id >> 5;
    for (auto& motor : motors_) {
        if (motor.id == node_id) {
            onReceive(msg, motor.odrive);
            return;
        }
    }
}

bool ODriveManager::setup_can() {
    g_instance = this;
    can_intf_.begin();
    can_intf_.setBaudRate(kCanBaudrate);
    can_intf_.setMaxMB(64);
    can_intf_.enableFIFO();
    can_intf_.enableFIFOInterrupt();
    can_intf_.onReceive(can_message_trampoline);
    return true;
}

bool ODriveManager::begin() {
    // Register ODrive-level callbacks before bringing up CAN so we don't
    // miss any frames that arrive during init.
    for (auto& motor : motors_) {
        motor.odrive.onFeedback(on_position_feedback, &motor.user_data);
        motor.odrive.onTorques(on_torque_feedback, &motor.user_data);
        motor.odrive.onStatus(on_heartbeat, &motor.user_data);
    }

    if (!setup_can()) {
        Serial.println("CAN failed to initialize: reset required");
        return false;
    }

    int c = 0;
    for (auto& motor : motors_) {
        motor.controller.set_ffwd_control(false);
        motor.controller.set_gvty_compensation(false);
        motor.controller.set_i_clamp_val(10.0);
        motor.controller.set_u_clamp_val(1.2);

        // Re-register (matches original behavior — harmless idempotent call).
        motor.odrive.onFeedback(on_position_feedback, &motor.user_data);
        motor.odrive.onTorques(on_torque_feedback, &motor.user_data);
        motor.odrive.onStatus(on_heartbeat, &motor.user_data);

        Serial.print("Waiting for ODrive");
        Serial.print(c);
        Serial.println("...");
        while (!motor.user_data.received_heartbeat) {
            pumpEvents(can_intf_);
        }
        Serial.print("found ODrive");
        Serial.print(c);
        Serial.println("...");

        Serial.println("attempting to read bus voltage and current");
        Get_Bus_Voltage_Current_msg_t vbus;
        if (!motor.odrive.request(vbus, 1000)) {
            Serial.println("vbus request failed!");
            return false;
        }
        Serial.print("DC voltage [V]: ");
        Serial.println(vbus.Bus_Voltage);
        Serial.print("DC current [A]: ");
        Serial.println(vbus.Bus_Current);

        Serial.print("Enabling closed loop control for Odrive");
        Serial.print(c);
        Serial.println("...");

        while (motor.user_data.last_heartbeat.Axis_State !=
               ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
            motor.odrive.clearErrors();
            delay(1);
            motor.odrive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            // ~150ms of pumping — see original comment for rationale
            // (need >=2 heartbeats; tolerate bus congestion on setState).
            for (int i = 0; i < 15; ++i) {
                delay(10);
                pumpEvents(can_intf_);
            }
        }
        Serial.print("ODrive");
        Serial.print(c);
        Serial.println(" ready!");
        c++;
    }
    return true;
}

void ODriveManager::can_loop() {
    can_intf_.events();  // drain all pending frames in one shot
}

void ODriveManager::control_loop() {
    for (size_t i = 0; i < motors_.size(); ++i) {
        switch (control_mode_) {
            case ODriveControlMode::CONTROL_MODE_POSITION_CONTROL:
                motors_[i].odrive.setPosition(commands[i]);
                break;
            case ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL:
                motors_[i].odrive.setVelocity(commands[i]);
                break;
            case ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL:
                motors_[i].odrive.setTorque(commands[i]);
                break;
            default:
                break;
        }
    }
}

void ODriveManager::set_control_mode(uint8_t control_mode, uint8_t input_mode) {
    control_mode_ = control_mode;
    input_mode_ = input_mode;

    for (auto& motor : motors_) {
        // Transition to IDLE and wait for heartbeat to confirm it.
        motor.odrive.setState(ODriveAxisState::AXIS_STATE_IDLE);
        while (motor.user_data.last_heartbeat.Axis_State !=
               ODriveAxisState::AXIS_STATE_IDLE) {
            delay(1);
            pumpEvents(can_intf_);
        }

        motor.odrive.setControllerMode(control_mode, input_mode);

        // Now re-enter closed loop.
        while (motor.user_data.last_heartbeat.Axis_State !=
               ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
            motor.odrive.clearErrors();
            delay(1);
            motor.odrive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
            for (int i = 0; i < 15; ++i) {
                delay(1);
                pumpEvents(can_intf_);
            }
        }
    }
}

std::array<float, ODriveManager::kNumMotors> ODriveManager::get_position_feedback() {
    std::array<float, kNumMotors> out;
    for (size_t i = 0; i < kNumMotors; ++i) {
        out[i] = motors_[i].user_data.last_position.Pos_Estimate;
    }
    return out;
}

std::array<float, ODriveManager::kNumMotors> ODriveManager::get_torque_feedback() {
    std::array<float, kNumMotors> out;
    for (size_t i = 0; i < kNumMotors; ++i) {
        out[i] = motors_[i].user_data.last_torque.Torque_Estimate;
    }
    return out;
}

float ODriveManager::get_active() {
    return active_;
}

void ODriveManager::set_active(float active) {
    active_ = active;
}

void ODriveManager::set_commands(const std::array<float, kNumMotors>& cmds) {
    commands = cmds;
}

std::array<float, ODriveManager::kNumMotors> ODriveManager::get_commands() const {
    return commands;
}

void ODriveManager::set_zero_position(const std::array<float, kNumMotors>& zero_pos) {
    for (size_t i = 0; i < kNumMotors; ++i) {
        auto current_pos = motors_[i].user_data.last_position.Pos_Estimate;
        motors_[i].odrive.setAbsolutePosition(current_pos - zero_pos[i]);
    }
}

void ODriveManager::set_velocity_limit(float velocity_limit) {
    for (size_t i = 0; i < kNumMotors; ++i) {
        motors_[i].odrive.setTrapezoidalVelLimit(velocity_limit);
    }
}

void ODriveManager::get_control_mode() {
    return control_mode_;
}