// Based on the following example:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
#include "ODriveCAN.h"
#include "pos_controller.hpp"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
#include <TeensyTimerTool.h>
#include <numbers>

#define CAN_BAUDRATE 250000
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 2

struct ODriveUserData
{
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

struct Motor
{
    ODriveCAN odrive;
    ODriveUserData user_data;
    PositionController controller;
};

struct ODriveStatus; // hack to prevent teensy compile error

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// auto controller = PositionController(.27, 0.012, 0.02);
// auto controller = PositionController(.2, 0.006, 0.008);
auto controller0 = PositionController(.0, 0.0, 0.0);
auto controller1 = PositionController(.0, 0.0, 0.0);
auto controller2 = PositionController(.0, 0.0, 0.0);

void onCanMessage(const CanMsg & msg);

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID); // Standard CAN message ID
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID); // Standard CAN message ID
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
Motor motor0 {odrv0, odrv0_user_data, controller0};
Motor motor1 {odrv1, odrv1_user_data, controller1};
Motor motor2 {odrv2, odrv2_user_data, controller2};
Motor * motors[] = {&motor0, &motor1, &motor2}; // Make sure all ODriveCAN instances are accounted for here

TeensyTimerTool::PeriodicTimer motor0_timer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer motor1_timer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer motor2_timer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer print_timer(TeensyTimerTool::TCK);

// init global vars
auto control_count = 0;
auto setpoint = 0.0;
auto actual = 0.0;
auto next = 0.0;
auto velocity = 0.0;
auto u = 0.0;
auto u_clamp = 0.15;
auto u_clamping = true;
auto encoder_offset = 0.0;
auto target = 0.0;
auto amplitude = 1.0;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t & msg, void * user_data)
{
  ODriveUserData * odrv_user_data = static_cast<ODriveUserData *>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t & msg, void * user_data)
{
  ODriveUserData * odrv_user_data = static_cast<ODriveUserData *>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg & msg)
{
  for (auto motor: motors) {
    onReceive(msg, motor->odrive);  // could be wrong
  }
}

bool setupCan()
{
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

void sin_control_loop(Motor motor)
{

  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
                        // Note that on MCP2515-based platforms, this will delay for a fixed 10ms.
                        //
                        // This has been found to reduce the number of dropped messages, however it can be removed
                        // for applications requiring loop times over 100Hz.

  float SINE_PERIOD = 2.50f; // Period of the position command sine wave in seconds

  float t = 0.001 * millis();

  float phase = t * (TWO_PI / SINE_PERIOD);

  setpoint = amplitude * sin(phase);
  actual = motor.user_data.last_feedback.Pos_Estimate - encoder_offset;
  next = amplitude * sin(phase + 0.001);
  velocity = motor.user_data.last_feedback.Vel_Estimate;

  u = motor.controller.pump_controller(setpoint, actual, next, odrv0_user_data.last_feedback.Vel_Estimate);

  motor.odrive.setTorque(u);
  
}

void step_control_loop(Motor motor)
{
  static double control = 1.0;

  pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages
                        // Note that on MCP2515-based platforms, this will delay for a fixed 10ms.
                        //
                        // This has been found to reduce the number of dropped messages, however it can be removed
                        // for applications requiring loop times over 100Hz.

  if (control_count > 1000)
  {
    control_count = 0;
    amplitude*=-1;
    control = target + amplitude;
  }

  setpoint = control;
  actual = motor.user_data.last_feedback.Pos_Estimate - encoder_offset;
  velocity = motor.user_data.last_feedback.Vel_Estimate;
  u = motor.controller.pump_controller(setpoint, actual, next, velocity);

  motor.odrive.setTorque(u);

  control_count++;
  
}

void print_loop()
{
  for (auto motor : motors)
    if (motor->user_data.received_feedback == true)
    {
      motor->user_data.received_feedback = false;
      Serial.print(">odrv0_pos:");
      Serial.print(actual);
      Serial.print("\n");
      // Serial.print(">cmd_pos:");
      // Serial.println(setpoint);
      // Serial.print("\n");
      // Serial.print(">u:");
      // Serial.println(u);
      // Serial.print("\n");
      Serial.print(">velocity:");
      Serial.println(velocity);
      Serial.print("\n");
      // Serial.print(">cos(angle):");
      // Serial.println(cos(2*PI*(actual) / 18.0));
      // Serial.print("\n");
      // Serial.print(">actual:");
      // Serial.println(actual);
      // Serial.print("\n");
      // Serial.print(">actual/gr:");
      // Serial.println(actual/18.0);
    }
}


void setup()
{

  Serial.begin(115200);
  Serial.println("Starting ODriveCAN demo");

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true) {} // spin indefinitely
  }

  auto c = 0;
  for (auto motor : motors) {
    // disable feed forward
    motor->controller.set_ffwd_control(false);
    motor->controller.set_gvty_compensation(false);
    motor->controller.set_i_clamp_val(10.0);
    motor->controller.set_u_clamp_val(1.2);
  

    // Register callbacks for the heartbeat and encoder feedback messages
    motor->odrive.onFeedback(onFeedback, &motor->user_data);
    motor->odrive.onStatus(onHeartbeat, &motor->user_data);
  
    
    Serial.print("Waiting for ODrive");
    Serial.print(c);
    Serial.println("...");
    while (!motor->user_data.received_heartbeat) {
      pumpEvents(can_intf);
    }

    Serial.print("found ODrive");
    Serial.print(c);
    Serial.println("...");
    // request bus voltage and current (1sec timeout)
    Serial.println("attempting to read bus voltage and current");
    Get_Bus_Voltage_Current_msg_t vbus;
    if (!motor->odrive.request(vbus, 1000)) {
      Serial.println("vbus request failed!");
      while (true) {} // spin indefinitely
    }

    Serial.print("DC voltage [V]: ");
    Serial.println(vbus.Bus_Voltage);
    Serial.print("DC current [A]: ");
    Serial.println(vbus.Bus_Current);

    Serial.println("Enabling closed loop control for Odrive");
    Serial.print(c);
    Serial.println("...");

    // set encoder offset
    encoder_offset = motor->user_data.last_feedback.Pos_Estimate;

    while (motor->user_data.last_heartbeat.Axis_State !=
      ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)
    {
      motor->odrive.clearErrors();
      delay(1);
      motor->odrive.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

      // Pump events for 150ms. This delay is needed for two reasons;
      // 1. If there is an error condition, such as missing DC power, the ODrive might
      //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
      //    on the first heartbeat response, so we want to receive at least two
      //    heartbeats (100ms default interval).
      // 2. If the bus is congested, the setState command won't get through
      //    immediately but can be delayed.
      for (int i = 0; i < 15; ++i) {
        delay(10);
        pumpEvents(can_intf);
      }
    }

    c++;
  }
  motor0_timer.begin(
    [](){
      step_control_loop(motor0);
    }, 1000);

  motor1_timer.begin(
    [](){
      step_control_loop(motor1);
    }, 1000);


  motor2_timer.begin(
    [](){
      step_control_loop(motor2);
    }, 1000);
  print_timer.begin(
    [](){
      print_loop();
    }, 1000);

  Serial.println("ODrive ready!");


}

void loop() {}
