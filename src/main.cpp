#include "Arduino.h"
#include "HardwareSerial.h"

extern "C"
{
#include "simpletuya.h"
}

#ifdef DEBUG
    #define LOG(data) { Serial.print(data); }
    #define LOGLN(data) { Serial.println(data); }
#else
    #define LOG(data) {}
    #define LOGLN(data) {}
#endif

#define HEALTHCHECK_INTERVAL 15000 // ms

typedef enum ParserState {
    STATE_HEADER_HIGH,
    STATE_HEADER_LOW,
    STATE_VERSION,
    STATE_COMMAND,
    STATE_DATA_LEN_HIGH,
    STATE_DATA_LEN_LOW,
    STATE_DATA,
    STATE_CHECKSUM
} ParserState;

HardwareSerial *serial = &Serial1;

void send_healthcheck(void);

void mcu_handler(void);

void write_data_frame(DataFrame *frame);

bool serial_read_frame(HardwareSerial *serial, BytesArray *buffer);

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif
    serial->begin(9600);
}

void loop() {
    send_healthcheck();
    mcu_handler();
}


void write_data_frame(DataFrame *frame) {
    BytesArray buffer;
    df2bytes(&buffer, frame);
#ifdef DEBUG
    char *str = bytes_array_to_str(&buffer);
    Serial.print("TX ");
    Serial.println(str);
    free(str);
#endif
    serial->write(buffer.bytes, buffer.len);
}


void send_healthcheck(void)
{
    static unsigned long last_healthcheck = 0;
    unsigned long current_time = millis();
    if (current_time > last_healthcheck + HEALTHCHECK_INTERVAL)
    {
        DataFrame respFrame;
        DataFrameDTO params = {
            .version=0x00,
            .command=CMD_HEALTHCHECK,
            .data_type=DT_EMPTY,
        };
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
        last_healthcheck = current_time;
    }
}


bool serial_read_frame(HardwareSerial *serial, BytesArray *dest)
{
    static ParserState state = STATE_HEADER_HIGH;
    static uint16_t data_len = 0;
    static uint16_t data_idx = 0;

    while (serial->available()) {
        uint8_t in_byte = serial->read();

        switch (state) {
            case STATE_HEADER_HIGH: {
                if (in_byte == 0x55) {
                    dest->bytes[dest->len++] = in_byte;
                    state = STATE_HEADER_LOW;
                } else {
                    dest->len = 0;
                }
                break;
            }

            case STATE_HEADER_LOW: {
                if (in_byte == 0xAA) {
                    state = STATE_VERSION;
                    dest->bytes[dest->len++] = in_byte;
                } else {
                    state = STATE_HEADER_HIGH;
                    dest->len = 0;
                }
                break;
            }

            case STATE_VERSION: {
                state = STATE_COMMAND;
                dest->bytes[dest->len++] = in_byte;
                break;
            }

            case STATE_COMMAND: {
                state = STATE_DATA_LEN_HIGH;
                dest->bytes[dest->len++] = in_byte;
                break;
            }

            case STATE_DATA_LEN_HIGH: {
                state = STATE_DATA_LEN_LOW;
                dest->bytes[dest->len++] = in_byte;
                data_len = in_byte << 8;
                break;
            }

            case STATE_DATA_LEN_LOW: {
                dest->bytes[dest->len++] = in_byte;
                data_len |= in_byte;

                if (data_len > DATA_BUFFER_SIZE - DF_MIN_SIZE) {
                    LOGLN("Data too long");
                    state = STATE_HEADER_HIGH;
                    data_len = 0;
                    dest->len = 0;
                } else if (data_len == 0) {
                    state = STATE_CHECKSUM;
                } else {
                    data_idx = 0;
                    state = STATE_DATA;
                }
                break;
            }

            case STATE_DATA: {
                dest->bytes[dest->len++] = in_byte;
                data_idx++;
                if (data_idx >= data_len) {
                    state = STATE_CHECKSUM;
                }
                break;
            }

            case STATE_CHECKSUM: {
                uint8_t expected_checksum = calculate_bytes_checksum(dest->bytes, dest->len);
                if (expected_checksum != in_byte) {
                    LOG("Invalid checksum: ");
                    LOGLN(in_byte);
                    LOG("Expected checksum: ");
                    LOGLN(expected_checksum);
                }
                dest->bytes[dest->len++] = in_byte;
#ifdef DEBUG
                char *str = bytes_array_to_str(dest);
                Serial.print("RX ");
                Serial.println(str);
                free(str);
#endif
                state = STATE_HEADER_HIGH;
                data_len = 0;
                return true;
            }
        }
    }
    return false;
}


void mcu_handler()
{
    static size_t lastState;
    static BytesArray buffer;
    static bool initialized = false;
    bool is_ready = serial_read_frame(serial, &buffer);
    if (!is_ready || buffer.len < DF_MIN_SIZE) { return; }
    DataFrame *frame = bytes2df(buffer.bytes, buffer.len);
    if (frame == NULL) { return; }
    switch (frame->command) {
        // <<--- Heartbeat check (0x00)
        case CMD_HEALTHCHECK: {
            if (frame->raw_data[0] == 0x00) {
                lastState = STATE_RESTARTED;
                // --->> Query product information (0x01)
                DataFrame respFrame;
                DataFrameDTO params = {
                    .version=0x00,
                    .command=CMD_QUERY_PROD_INFO,
                    .data_type=DT_EMPTY,
                };
                init_data_frame(&respFrame, &params);
                write_data_frame(&respFrame);
            } else {
                lastState = STATE_RUNNING;
                if (!initialized) {
                    initialized = true;
                    DataFrame respFrame;
                    DataFrameDTO params = {
                        .version=0x00,
                        .command=CMD_QUERY_STATUS,
                        .data_type=DT_EMPTY,
                    };
                    init_data_frame(&respFrame, &params);
                    write_data_frame(&respFrame);

                }
            }
            break;
        }
        // <<--- Query product information (0x01)
        case CMD_QUERY_PROD_INFO: {
            /*
             * there is no need to handle this response,
             * just send another request from the initialization chain
             */

            // --->> Query Working mode (0x02)
            DataFrame respFrame;
            DataFrameDTO params = {
                .version=0x00,
                .command=CMD_WORKING_MODE,
                .data_type=DT_EMPTY,
            };
            init_data_frame(&respFrame, &params);
            write_data_frame(&respFrame);
            break;
        }
        // <<--- Query working mode (0x02)
        case CMD_WORKING_MODE: {
            /*
             * there is no need to handle this response,
             * just send another request from the initialization chain
             */

            // --->> Report network status (0x03)
            DataFrame respFrame;
            BytesArray networkStatusData = {.len=1, .bytes={STATUS_5}};
            DataFrameDTO params = {
                .version=0x00,
                .command=CMD_NETWORK_STATUS,
                .data_type=DT_RAW,
            };
            params.raw_data=&networkStatusData;
            init_data_frame(&respFrame, &params);
            write_data_frame(&respFrame);
            break;
        }
            // <<--- Report network status (0x03)
        case CMD_NETWORK_STATUS: {
            /*
             * there is no need to handle this response,
             * just send another request from the initialization chain
             */

            // --->> Heartbeat check after the initialization process
            DataFrame respFrame;
            DataFrameDTO params = {
                .version=0x00,
                .command=CMD_HEALTHCHECK,
                .data_type=DT_EMPTY,
            };
            init_data_frame(&respFrame, &params);
            write_data_frame(&respFrame);
            break;
        }

        case CMD_RESET_MODULE: {
            asm volatile ("jmp 0");
            break;
        }
    }
    free_data_frame(frame);
    memset(&buffer, 0, sizeof(BytesArray));
}


