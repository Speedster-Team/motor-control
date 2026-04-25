// #include "TeensyTimerTool.h"
// #include <Arduino.h>

// // maximum message size 10 seconds of commands = 64128 bytes total
// #define MAX_LINE_LENGTH 64
// #define MAX_MESSAGE_LENGTH 1002
// #define NUM_MOTORS 3

// struct Command {
//     char type;
//     int length;
//     int repeat;
// };

// TeensyTimerTool::PeriodicTimer serialTimer(TeensyTimerTool::TCK);

// char lineBuf[MAX_LINE_LENGTH];
// char message[MAX_MESSAGE_LENGTH][MAX_LINE_LENGTH];
// float positions[MAX_MESSAGE_LENGTH][NUM_MOTORS];
// volatile bool messageReady = false;
// volatile int messageLineCount = 0;
// volatile int lineBufPos = 0;
// auto cmd = Command{' ', 0, 0};
// auto flag_error = 0;
// uint8_t checksum_recieved = 0;
// uint8_t checksum_computed = 0;
// uint8_t crc8_table[256];

// void buildCRC8Table() {
//     for (int i = 0; i < 256; i++) {
//         uint8_t crc = i;
//         for (int b = 0; b < 8; b++) {
//             crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
//         }
//         crc8_table[i] = crc;
//     }
// }

// uint8_t crc8_message(char data[][MAX_LINE_LENGTH], int start, int end) {
//     uint8_t crc = 0x00;
//     for (int i = start; i < end; i++) {
//         const uint8_t* line = (uint8_t*)data[i];
//         size_t len = strlen(data[i]);  // could fail if /0 is removed during strcpy 
//         for (size_t j = 0; j < len; j++) {
//             crc = crc8_table[crc ^ line[j]];
//         }
//     }
//     return crc;
// }

// void parse_header_line(char* line, int lineIdx) {
//     // get first char
//     char* token = strtok(line, " ");
//     cmd.type = token[0];  // char indicating command type
//     token = strtok(nullptr, " ");
//     cmd.length = atoi(token);  // command length 
//     token = strtok(nullptr, " ");
//     cmd.repeat = atoi(token);  // command repeat?
// }

// void parse_data_line(char* line, int lineIdx) {
//     int motorIdx = 0;
//     char* token = strtok(line, " ");
//     while (token != nullptr && motorIdx < NUM_MOTORS) {
//         positions[lineIdx][motorIdx++] = atof(token);
//         token = strtok(nullptr, " ");
//     }
//     if (motorIdx != NUM_MOTORS) {
//         // wrong number of values on this line — flag error
//         flag_error = 2;
//     }
// }

// void parse_footer_line(char* line) {
//     checksum_recieved = (uint8_t)atoi(line);  // the whole line is just the checksum number
// }

// void response() {
//     Serial.printf("%c %d %d %d %d %d\n", cmd.type, messageLineCount, cmd.repeat, checksum_recieved, checksum_computed, flag_error);
// }

// void serial_loop() {
    
// }

// void setup() {
//     Serial.begin(115200);
//     buildCRC8Table();
//     // serialTimer.begin(serial_loop, 1000);    // 1kHz - check serial every 1ms
// }

// void loop() {

// }