# rds_1dof
Code for RDS one DOF prototype, communicating with Odrive pro over CAN.
This repo is intended for Teensy 4.1 but will also work on Teensy 4.0 with some adjustments.

### Start Up 
0. Use [Odrive WebGui](https://gui.odriverobotics.com/configuration) to configure odrive for torque control
1. Clone repo into a PlatformIO workspace. 
2. Build and upload to teensy 4.1
3. Use Teleplot to get real time feedback on controller performance

### RDS Group Notes
We are using [this](https://aifitlab.com/products/dji-robomaster-m2006-p36-brushless-dc-gear-motor) motor
With [this](https://www.digikey.com/en/products/detail/ams-osram-usa-inc/AS5048B-ADAPTERBOARD/3188613) SPI encoder


### Laptop - Teensy communication standard

#### Data command from laptop to teensy
This command is used to send a position trajectory for the teensy to send to motors.

    <command_type = D> <data_length: int, 1-1000> <repeat = 1 | 0>

    <mcp_splay_position: float> <mcp_flex_position: float> <pip_flex_position: float>

    ...

    <mcp_splay_position: float> <mcp_flex_position: float> <pip_flex_position: float>
            
    <data_crc8: uint_8> 
    end

#### Go command from laptop to teensy
This command will start the teensy tracking the trajectory.

    <command_type = G>
    end


#### Stop command from laptop to teensy
This command will stop the teensy safely.

    <command_type = S>
    end

#### General feedback from teensy to laptop
The teensy is constantly sending these commands back to the laptop.

    <mcp_splay_position: float> <mcp_flex_position: float> <pip_flex_position: float> <active_bool: 1.0 | 0.0 >

#### Message recieved feedback from teensy to laptop
This message is response for a message recieved from the laptop.

    <success_bool >


