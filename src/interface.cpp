// SerialReceiver.cpp
#include "interface.hpp"

SerialInterface::SerialInterface(usb_serial_class serial)
    : _message_ready(false)
    , _message_line_count(0)
    , _line_buf_pos(0)
    , _cmd{' ', 0, 0}
    , _flag_error(0)
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
        _flag_error = 2;
}

void SerialInterface::parse_footer_line(char* line) {
    _checksum_received = (uint8_t)atoi(line);
}

void SerialInterface::response() {
    Serial.printf("%c %d %d %d %d %d\n",
        _cmd.type, _message_line_count, _cmd.repeat,
        _checksum_received, _checksum_computed, _flag_error);
}

void SerialInterface::process_message() {
    parse_footer_line(_message[_message_line_count - 1]);
    _checksum_computed = crc8_message(1, _message_line_count - 1);
    parse_header_line(_message[0]);
    for (int i = 1; i < _message_line_count - 1; i++)
        parse_data_line(_message[i], i - 1);
    if (_checksum_computed != _checksum_received)
        _flag_error = 99;
    response();
    _message_ready = false;
    _message_line_count = 0;
    _flag_error = 0;
}

void SerialInterface::loop() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            _line_buf[_line_buf_pos] = '\0';
            if (strcmp(_line_buf, "end") == 0)
                _message_ready = true;
            else
                strncpy(_message[_message_line_count++], _line_buf, MAX_LINE_LENGTH - 1);
            _line_buf_pos = 0;
        } else if (_line_buf_pos < MAX_LINE_LENGTH - 1) {
            _line_buf[_line_buf_pos++] = c;
        }
    }
    if (_message_ready)
        process_message();
}

Command SerialInterface::get_command() const {
    return _cmd;
}