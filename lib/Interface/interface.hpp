#ifndef INTERFACE_HPP
#define INTERFACE_HPP
#include <Arduino.h>

#define MAX_LINE_LENGTH 64
#define MAX_MESSAGE_LENGTH 1002
#define NUM_MOTORS 3

struct Command {
    char type;
    int length;
    int repeat;
    char mode;
};

class SerialInterface
{
public:
    SerialInterface(usb_serial_class serial);

    void build_CRC8_table();

    uint8_t crc8_message(int start, int end);

    void parse_header_line(char* line);

    void parse_data_line(char* line, int lineIdx);

    void parse_footer_line(char* line);

    void response();

    void feedback(float mcp_splay_motor_pos, float mcp_flex_motor_pos, float pip_flex_motor_pos, float cmd_mcp_splay_motor_pos, float cmd_mcp_flex_motor_pos, float cmd_pip_flex_motor_pos, float active);

    void process_message();

    void loop();

    Command get_command() const;

    void set_command_type(Command type);

    float _positions[MAX_MESSAGE_LENGTH][NUM_MOTORS];

private:
    bool _message_ready;
    int _message_line_count;
    int _line_buf_pos;
    Command _cmd;
    int _flag_error;
    uint8_t _checksum_received;
    uint8_t _checksum_computed;
    usb_serial_class _user_serial;
    uint8_t _crc8_table[256];
    char _line_buf[MAX_LINE_LENGTH];
    char _message[MAX_MESSAGE_LENGTH][MAX_LINE_LENGTH];
};

#endif