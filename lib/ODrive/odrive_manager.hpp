#pragma once

#include "ODriveCAN.h"
//#include "ODriveFlexCAN.hpp"
#include "pos_controller.hpp"
#include "interface.hpp"
#include <FlexCAN_T4.h>
#include <array>

using CanMsg = CAN_message_t;

struct ODriveStatus; // hack to prevent teensy compile error

struct ODriveUserData {
    Heartbeat_msg_t last_heartbeat;
    bool received_heartbeat = false;
    Get_Encoder_Estimates_msg_t last_position;
    bool received_position = false;
    Get_Torques_msg_t last_torque;
    bool received_torque = false;
};

// Replace the Motor constructor definition with just a declaration:
struct Motor {
    ODriveCAN odrive;
    ODriveUserData user_data;
    PositionController controller;
    int id;

    Motor(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& can_intf, int node_id);
};

class ODriveManager {
public:
    static constexpr int kNumMotors = 3;
    static constexpr uint32_t kCanBaudrate = 250000;

    explicit ODriveManager();

    // Brings up CAN, registers callbacks, waits for heartbeats, requests
    // bus voltage, and transitions every motor into closed-loop control.
    // Blocks until all motors are ready. Returns false if CAN init fails.
    bool begin();

    // Per-loop entry points — call these from your timer callbacks.
    void can_loop();
    void control_loop();

    // CAN frame router — must be installed as the FlexCAN_T4 onReceive
    // callback. Public because the callback registration needs it.
    void on_can_message(const CanMsg& msg);

    // Change Control Mode
    void set_control_mode(uint8_t control_mode, uint8_t input_mode);

    // Postition Feedback
    std::array<float, kNumMotors> get_position_feedback();

    // Torque Feedback
    std::array<float, kNumMotors> get_torque_feedback();
    
    // Get and Set for Active
    float get_active();
    void set_active(float active);

    // Get and Set for command
    std::array<float, kNumMotors> get_commands() const;
    void set_commands(const std::array<float, kNumMotors>& cmds);

    void set_zero_position(const std::array<float, kNumMotors>& zero_pos);

    void set_velocity_limit(float velocity_limit);


private:
    // ODrive callbacks (free-function signatures, take user_data ptr).
    static void on_heartbeat(Heartbeat_msg_t& msg, void* user_data);
    static void on_position_feedback(Get_Encoder_Estimates_msg_t& msg, void* user_data);
    static void on_torque_feedback(Get_Torques_msg_t& msg, void* user_data);

    bool setup_can();

    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf_;
    std::array<Motor, kNumMotors> motors_;

    uint8_t control_mode_ = ODriveControlMode::CONTROL_MODE_POSITION_CONTROL;
    uint8_t input_mode_ = ODriveInputMode::INPUT_MODE_PASSTHROUGH;

    volatile float active_ = 0.0f;
    std::array<float, kNumMotors> commands = {0.0f};
};