#include "Arduino.h"
#include "HardwareSerial.h"

extern "C"
{
#include "simpletuya.h"
}


#define HEALTHCHECK_INTERVAL 15000 // ms

HardwareSerial *serial = &Serial1;

void send_healthcheck(void);
void mcu_handler(void);
void write_data_frame(DataFrame *frame);

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

void read_serial(BytesArray *buf) {
    size_t i = 0;
    if (serial->available() > 0){
        delayMicroseconds(800);
    }
    while (serial->available()) {
        uint8_t in_byte = serial->read();
        buf->bytes[++i] = in_byte;
        delayMicroseconds(800);
    }
    buf->len = i;

#ifdef DEBUG
    if (buf->len > 1) {
        char *str = bytes_array_to_str(buf);
        Serial.print("RX ");
        Serial.println(str);
        free(str);
    }
#endif
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


void mcu_handler() {
    static size_t lastState;
    BytesArray buffer;
    memset(&buffer, 0, sizeof(BytesArray));
    read_serial(&buffer);
    if (buffer.len < DF_MIN_SIZE) { return; }
    DataFrame *frame = bytes2df(buffer.bytes, buffer.len);
    if (frame == NULL) { return; }
#ifdef DEBUG
    if (is_frame_valid(frame)) {
        Serial.println("VALID");
    } else {
        Serial.println("INVALID");
    }
#endif


    switch (frame->command) {
        // <<--- Heartbeat check (0x00)
        case CMD_HEALTHCHECK: {
            if (frame->raw_data[0] == 0x00) {
                lastState = STATE_RESTARTED;
            } else {
                lastState = STATE_RUNNING;
            }
            // --->> Query product information (0x01)
            DataFrame respFrame;
            DataFrameDTO params = {
                .version=0x03,
                .command=CMD_QUERY_PROD_INFO,
                .data_type=DT_EMPTY,
            };
            init_data_frame(&respFrame, &params);
            write_data_frame(&respFrame);
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
                .version=0x03,
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
            BytesArray networkStatusData = {.len=1, .bytes={STATUS_4}};
            DataFrameDTO params = {
                .version=0x03,
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
                .version=0x03,
                .command=CMD_HEALTHCHECK,
                .data_type=DT_EMPTY,
            };
            init_data_frame(&respFrame, &params);
            write_data_frame(&respFrame);
            break;
        }
    }
    free_data_frame(frame);
}


