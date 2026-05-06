// SerialReceiver.cpp
#include "interface.hpp"

SerialInterface::SerialInterface(usb_serial_class serial)
    : _message_ready(false)
    , _message_line_count(0)
    , _line_buf_pos(0)
    , _cmd{' ', 0, 0}
    , _flag_error(1)
    , _checksum_received(0)
    , _checksum_computed(0)
    , _user_serial(serial)
{
    build_CRC8_table();
}

void SerialInterface::build_CRC8_table() {
    for (int i = 0; i < 256; i++) {
        uint8_t crc = i;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        _crc8_table[i] = crc;
    }
}

uint8_t SerialInterface::crc8_message(int start, int end) {
    uint8_t crc = 0x00;
    for (int i = start; i < end; i++) {
        const uint8_t* line = (uint8_t*)_message[i];
        size_t len = strlen(_message[i]);
        for (size_t j = 0; j < len; j++)
            crc = _crc8_table[crc ^ line[j]];
    }
    return crc;
}

void SerialInterface::parse_header_line(char* line) {
    char* token = strtok(line, " ");
    _cmd.type = token[0];

    // check for stop command
    if (_cmd.type == 'S') {
        Serial.println("Received stop command");
        // stop command
        _cmd.length = 0;
        _cmd.repeat = 0;
        return;
    } else if (_cmd.type == 'G') {
        Serial.println("Received go command");
        // go command
        return;
    }
    token = strtok(nullptr, " ");
    _cmd.length = atoi(token);
    token = strtok(nullptr, " ");
    _cmd.repeat = atoi(token);
}

void SerialInterface::parse_data_line(char* line, int lineIdx) {
    int motorIdx = 0;
    char* token = strtok(line, " ");
    while (token != nullptr && motorIdx < NUM_MOTORS) {
        _positions[lineIdx][motorIdx++] = atof(token);
        token = strtok(nullptr, " ");
    }
    if (motorIdx != NUM_MOTORS)
        _flag_error = 0;
}

void SerialInterface::parse_footer_line(char* line) {
    _checksum_received = (uint8_t)atoi(line);
}

void SerialInterface::response() {
    Serial.printf("%d\n", _flag_error);
}

void SerialInterface::feedback(float mcp_splay_motor_pos, float mcp_flex_motor_pos, float pip_flex_motor_pos, float active) {
    Serial.printf("%f %f %f %f\n", mcp_splay_motor_pos, mcp_flex_motor_pos, pip_flex_motor_pos, active);
}

void SerialInterface::process_message() {
    // check type of command
    parse_header_line(_message[0]);

    if (_cmd.type == 'D'){
        // get crc
        parse_footer_line(_message[_message_line_count - 1]);

        // compute crc on this side
        _checksum_computed = crc8_message(1, _message_line_count - 1);

        // save data
        for (int i = 1; i < _message_line_count - 1; i++)
        {
            parse_data_line(_message[i], i - 1);
        }

        // check checksum
        if (_checksum_computed != _checksum_received)
        {
            _flag_error = 0;
        }
        
        // send response
        response();

        // reset vars
        _message_ready = false;
        _message_line_count = 0;
        _flag_error = 1;
    } else {
        response();
        _message_ready = false;
        _message_line_count = 0;
        _flag_error = 1;  // should already be 1
    }
}

void SerialInterface::loop() {
    // empty out buffer
    while (Serial.available()) {

        // get next char
        char c = Serial.read();

        // check if it is a new line char
        if (c == '\n') {

            // replace with null terminator
            _line_buf[_line_buf_pos] = '\0';

            // check if it is the last line
            if (strcmp(_line_buf, "end") == 0)
            {
                // if so don't save 'end' and flip bool so message is processed
                _message_ready = true;
            }
            else
            {
                // otherwise save row and continue looking for end
                // strncpy(_message[_message_line_count++], _line_buf, MAX_LINE_LENGTH - 1);
                snprintf(_message[_message_line_count++], MAX_LINE_LENGTH, "%s", _line_buf);
            }
            _line_buf_pos = 0;
        } else if (_line_buf_pos < MAX_LINE_LENGTH - 1) {
            _line_buf[_line_buf_pos++] = c;
        }
    }
    if (_message_ready)
    {
        Serial.printf("%s\n", _message[0]);
        process_message();
    }
}

Command SerialInterface::get_command() const {
    return _cmd;
}

void SerialInterface::set_command_type(Command type) {
    _cmd.type = type.type;
}