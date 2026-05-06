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
 
void control_loop() {
    auto cmd = interface.get_command();
    static int control_count_ = 0;

    //Serial.printf("Received command: type=%c, length=%d, repeat=%d\n", cmd.type, cmd.length, cmd.repeat);

    if (cmd.type == 'G' && control_count_ < cmd.length) {
        odrive_mgr.set_active(1.0f);

        std::array<float, odrive_mgr.kNumMotors> motor_cmd = {
          interface._positions[control_count_][0]/ 6.28f, 
          interface._positions[control_count_][1]/ 6.28f, 
          interface._positions[control_count_][2]/ 6.28f};

        odrive_mgr.set_commands(motor_cmd);

        control_count_++;
        if (control_count_ == cmd.length && cmd.repeat == 1) {
            control_count_ = 0;
        }
    } else if (cmd.type == 'S') {
        odrive_mgr.set_active(0.0f);
        // Stop semantics TBD — matches original (no-op on motors).
        control_count_ = 0;
    } else {
        odrive_mgr.set_active(0.0f);
        control_count_ = 0;

        auto cmd = Command{'A', 0, 0};
        interface.set_command_type(cmd);

    }
}

void pass_feedback() {
    auto fb = odrive_mgr.get_position_feedback();
    interface.feedback(fb[0], fb[1], fb[2], odrive_mgr.get_active());
}

void startup_procedure() {

  Serial.println("Starting startup procedure...");
  
  odrive_mgr.set_control_mode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH); 
  //float torque_threshold = 0.15f; // empirically determined threshold for "motor is ready"
  std::array<float, 3> zero_angles = {-0.611f/6.28f, 0.154f/6.28f, 0.274f/6.28f};

  // pip-dip 0
  Serial.println("splay 0");
  std::array<float, 3> cmd = {0.1, 0.0f, 0.0f};
  odrive_mgr.set_commands(cmd);

  while(fabsf(odrive_mgr.get_torque_feedback()[0] < 0.1)) {
    delay(5);
  }

  // mcp 0
  Serial.println("mcp 0");
  std::array<float, 3> cmd_2 = {0.0, 0.1f, 0.0f};
  odrive_mgr.set_commands(cmd_2);

  while(fabsf(odrive_mgr.get_torque_feedback()[1] < 0.15)) {
    delay(5);
  }

  // splay 0
  Serial.println("pip/dip 0");
  std::array<float, 3> cmd_3 = {0.0f, 0.0f, 0.1f};
  odrive_mgr.set_commands(cmd_3);

  while(fabsf(odrive_mgr.get_torque_feedback()[2] < 0.08)) {
    delay(5);
  }
  auto encoder_offset = odrive_mgr.get_position_feedback();
  auto hi = std::array<float, 3>{encoder_offset[0] - zero_angles[0],encoder_offset[1] - zero_angles[1],encoder_offset[2] - zero_angles[2]};
  odrive_mgr.set_zero_position(hi);

  std::array<float, 3> cmd_zero = {0.0f, 0.0f, 0.0f};
  odrive_mgr.set_commands(cmd_zero);
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

    interface_timer.begin([]() {interface.loop();}, 1000);
    feedback_timer.begin([]() { pass_feedback(); }, 10000);
    control_timer.begin([]() { control_loop(); }, 10000);
}

void loop() {}