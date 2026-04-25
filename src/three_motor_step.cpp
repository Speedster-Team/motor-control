// Based on the following example:
// https://docs.odriverobotics.com/v/latest/guides/arduino-can-guide.html
#include "ODriveCAN.h"
#include "pos_controller.hpp"
#include "interface.hpp"
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
    ODriveCAN& odrive;
    ODriveUserData user_data;
    PositionController controller;
    int id;
};

struct ODriveStatus; // hack to prevent teensy compile error

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

auto interface = SerialInterface(Serial);


void onCanMessage(const CanMsg & msg);

ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID); // Standard CAN message ID
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID); // Standard CAN message ID

Motor motor0 {odrv0,
              ODriveUserData(),
              PositionController(.0, 0.0, 0.0),
              ODRV0_NODE_ID};
Motor motor1 {odrv1,
              ODriveUserData(),
              PositionController(.0, 0.0, 0.0),
              ODRV1_NODE_ID};
Motor motor2 {odrv2,
              ODriveUserData(),
              PositionController(.0, 0.0, 0.0),
              ODRV2_NODE_ID};
Motor * motors[] = {&motor0, &motor1, &motor2}; // Make sure all ODriveCAN instances are accounted for here

TeensyTimerTool::PeriodicTimer motor_timer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer interface_timer(TeensyTimerTool::TCK);

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

// // Called for every message that arrives on the CAN bus
// void onCanMessage(const CanMsg& msg) {
//   for (auto motor: motors) {
//     onReceive(msg, motor->odrive);
//   }
// }

// Route CAN by node ID:
void onCanMessage(const CanMsg& msg) {
    uint8_t node_id = msg.id >> 5;
    for (auto motor : motors) {
        if (motor->id == node_id) {
            onReceive(msg, motor->odrive);
            return;
        }
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

void control_loop()
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

  for (auto motor : motors) {
    motor->odrive.setPosition(control);
  }

  control_count++;
  
}


void control_loop2()
{
  auto cmd = interface.get_command();
  // printf("%c %d %d\n", cmd.type, cmd.length, cmd.repeat);

  pumpEvents(can_intf);

  if (cmd.type == 's' && control_count < cmd.length) {

 
    auto motor_count = 0;
    for (auto motor : motors) {
      motor->odrive.setPosition(interface._positions[control_count][motor_count]);
      motor_count++;
    }

    control_count++;

    if (control_count == cmd.length && cmd.repeat == 1){
      control_count = 0;
    }

  }

  
}
void setup()
{

  Serial.begin(115200);
  while(!Serial){}
  Serial.println("Starting ODriveCAN demo");

  // init count
  auto c = 0;

  // must register callbacks before initializing motors
  for (auto motor : motors) {
    // Register callbacks for the heartbeat and encoder feedback messages
    motor->odrive.onFeedback(onFeedback, &motor->user_data);
    motor->odrive.onStatus(onHeartbeat, &motor->user_data);
  }

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true) {} // spin indefinitely
  }

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

    Serial.print("Enabling closed loop control for Odrive");
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
    Serial.print("ODrive");
    Serial.print(c);
    Serial.println(" ready!");

    c++;
  }

  interface_timer.begin(
    [](){
      interface.loop();
    }, 1000);

    motor_timer.begin(
    [](){
      control_loop2();
    }, 10000);

}

void loop() {}
