#include <Arduino.h>
#include <TeensyTimerTool.h>
#include "interface.hpp"
#include "odrive_manager.hpp"
 
static SerialInterface interface(Serial);
static ODriveManager odrive_mgr;
 
static TeensyTimerTool::PeriodicTimer control_timer(TeensyTimerTool::TCK);
static TeensyTimerTool::PeriodicTimer motor_timer(TeensyTimerTool::TCK);
static TeensyTimerTool::PeriodicTimer interface_timer(TeensyTimerTool::TCK);
static TeensyTimerTool::PeriodicTimer can_timer(TeensyTimerTool::TCK);
static TeensyTimerTool::PeriodicTimer feedback_timer(TeensyTimerTool::TCK);
static TeensyTimerTool::PeriodicTimer mode_timer(TeensyTimerTool::TCK);

static volatile int control_count_ = 0;
static volatile float rotation_conversion_ = 6.28f;

static std::array<float, 3> last_setpoint_ = {0.f, 0.f, 0.f};

static Command last_cmd_{' ', 0, 0, ' '};

void control_loop() {
    auto cmd = interface.get_command();

    if (last_cmd_.mode != cmd.mode) {
        switch (cmd.mode) {
            case 'P':
                odrive_mgr.set_control_mode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
                rotation_conversion_ = 6.28f;
                break;
            case 'V':
                odrive_mgr.set_control_mode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
                rotation_conversion_ = 6.28f;
                break;
            case 'T':
                odrive_mgr.set_control_mode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
                rotation_conversion_ = 1.0f;
                break;
            default:
                // default to position control if mode is unrecognized
                odrive_mgr.set_control_mode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
        }
        last_cmd_ = cmd;
    }
    
    if (cmd.type == 'G' && control_count_ < cmd.length) {
        odrive_mgr.set_active(1.0f);

        std::array<float, odrive_mgr.kNumMotors> motor_cmd = {
          interface._positions[control_count_][0] / rotation_conversion_, 
          interface._positions[control_count_][1] / rotation_conversion_, 
          interface._positions[control_count_][2] / rotation_conversion_};

        odrive_mgr.set_commands(motor_cmd);

        // Latch setpoint in radians (what you actually sent)
        last_setpoint_ = {
            interface._positions[control_count_][0],
            interface._positions[control_count_][1],
            interface._positions[control_count_][2]
        };

        control_count_++;
        if (control_count_ == cmd.length && cmd.repeat == 1) {
            control_count_ = 0;
        }
    } else if (cmd.type == 'S') {
        odrive_mgr.set_active(0.0f);
        control_count_ = 0;

        // hold position after stopping
        odrive_mgr.set_control_mode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
        auto current_pos = odrive_mgr.get_position_feedback();
        odrive_mgr.set_commands(current_pos);

        auto cmd = Command{'A', 0, 0};
        interface.set_command_type(cmd);
    } else {
        odrive_mgr.set_active(0.0f);
        control_count_ = 0;

        auto cmd = Command{'A', 0, 0};
        interface.set_command_type(cmd);

    }
}

void pass_feedback() {
    auto fb = odrive_mgr.get_position_feedback();

    interface.feedback(
    fb[0] * 6.28f, fb[1] * 6.28f, fb[2] * 6.28f,
    last_setpoint_[0], last_setpoint_[1], last_setpoint_[2],
    odrive_mgr.get_active()
    );
}

void startup_procedure() {

  Serial.println("Starting startup procedure...");
  
  odrive_mgr.set_control_mode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH); 
  //float torque_threshold = 0.15f; // epirically determined threshold for "motor is ready"
  std::array<float, 3> zero_angles = {-1.68f/6.28f, 1.22/6.28f, -0.896f/6.28f};
//   std::array<float, 3> zero_angles = {0.0f/6.28f, 0.0/6.28f, 0.0f/6.28f};

  // pip-dip 0
  Serial.println("splay 0");
  std::array<float, 3> cmd = {-0.2, 0.0f, 0.0f};
  odrive_mgr.set_commands(cmd);

  while(fabsf(odrive_mgr.get_torque_feedback()[0]) < 0.3) {
    delay(5);
  }
  // mcp 0
  Serial.println("mcp 0");
  std::array<float, 3> cmd_2 = {0.0, 0.2f, 0.0f};
  odrive_mgr.set_commands(cmd_2);

  while(fabsf(odrive_mgr.get_torque_feedback()[1]) < 0.15) {
    delay(5);
  }

  // splay 0
  Serial.println("pip/dip 0");
  std::array<float, 3> cmd_3 = {-0.05f, 0.0f, 0.2f};
  odrive_mgr.set_commands(cmd_3);

  while(fabsf(odrive_mgr.get_torque_feedback()[2]) < 0.08) {
    delay(5);
  }
  auto encoder_offset = odrive_mgr.get_position_feedback();
  auto hi = std::array<float, 3>{encoder_offset[0] - zero_angles[0],encoder_offset[1] - zero_angles[1],encoder_offset[2] - zero_angles[2]};
  odrive_mgr.set_zero_position(hi);


  std::array<float, 3> cmd_zero = {0.0f, 0.0f, 0.0f};
  odrive_mgr.set_velocity_limit(0.5f);
  odrive_mgr.set_control_mode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_TRAP_TRAJ);
  odrive_mgr.set_commands(cmd_zero);
  delay(2000);
  odrive_mgr.set_velocity_limit(50.0f);
  odrive_mgr.set_control_mode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);

  Serial.println("Calibration procedure complete!");

}

void setup() {
    Serial.begin(115200);
    // while (!Serial) {}
    Serial.println("Starting ODriveCAN demo");
 
    if (!odrive_mgr.begin()) {
        while (true) {}  // spin indefinitely on init failure
    }
 
    can_timer.begin([]() { odrive_mgr.can_loop(); }, 1000);
    motor_timer.begin([]() { odrive_mgr.control_loop(); }, 10000);

    // Startup procedure
    startup_procedure();

    interface_timer.begin([]() { interface.loop(); }, 1000);
    feedback_timer.begin([]() { pass_feedback(); }, 10000);
    control_timer.begin([]() { control_loop(); }, 10000);
}

void loop() {}