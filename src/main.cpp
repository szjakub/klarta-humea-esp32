#include "Arduino.h"
#include <SPI.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include "HardwareSerial.h"
#include <cstdint>
#include <string.h>
#include <klarta.h>
#include <secrets.h>

extern "C"
{
#include "simpletuya.h"
}

#ifdef DEBUG
    #define LOG(data) { Serial.print(data); }
    #define LOGLN(data) { Serial.println(data); }
    #define LOGBUF(buffer, prefix)                  \
    ({                                              \
        char *str = bytes_array_to_str(&buffer);    \
        Serial.print(prefix);                       \
        Serial.print(" ");                          \
        Serial.println(str);                        \
        free(str);                                  \
    })
#else
    #define LOG(data) {}
    #define LOGLN(data) {}
    #define LOGBUF(buffer, prefix) {}
#endif

#define RX_1 12
#define TX_1 13

void logf(const char *format, ...);
int ascii_bytes_to_int(byte *payload, unsigned int length, int fallback);
void mqtt_callback(char *topic, byte *payload, unsigned int length);
void mqtt_handler(char *topic, byte *payload, unsigned int length);
void send_data(DataFrame *frame);
void send_healthcheck(void);
void mcu_handler(HardwareSerial *serial);
void write_data_frame(DataFrame *frame);
bool serial_read_frame(HardwareSerial *serial, BytesArray *buffer);


void logf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    LOG(buffer);
}

HardwareSerial *serial = &Serial1;
IPAddress ha_server(192, 168, 1, 21);
WiFiClient wifi_client;
PubSubClient mqtt_client(ha_server, 1883, mqtt_callback, wifi_client);

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif
    serial->begin(9600, SERIAL_8N1, RX_1, TX_1);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    WiFi.setHostname(HOSTNAME);
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        LOG('.');
    }

    if(mqtt_client.connect("arduinoClient", "klarta", "password1")) {
        mqtt_client.subscribe(TOPIC_STATE_SET);
        mqtt_client.subscribe(TOPIC_FAN_SET);
    }
}

void loop() {
    mqtt_client.loop();
    send_healthcheck();
    mcu_handler(serial);
}

int ascii_bytes_to_int(byte *payload, unsigned int length, int fallback)
{
    if (length > 10) {
        logf("Value of length %d is too big\n", length);
        return fallback;
    }
    char buffer[11];

    for (int i=0; i<length; i++) {
        byte value = payload[i];
        if (value < 0x30 || value > 0x39) {
            logf("ASCII value of %d is not a number\n", (unsigned char)value);
            return fallback;
        }
        buffer[i] = (char)payload[i];
    }
    buffer[length] = '\0';
    return atoi(buffer);
}

void mqtt_callback(char *topic, byte *payload, unsigned int length) {
    if (strcmp(topic, TOPIC_STATE_SET) == 0) {
        uint8_t state = (uint8_t)ascii_bytes_to_int(payload, length, 0);
        DataUnit du;
        DataUnitDTO du_params = {
            .dpid=DPID_STATE,
            .type=TYPE_BOOL,
            .byte_value=state
        };
        init_data_unit(&du, &du_params);
        DataFrame respFrame;
        DataFrameDTO params = {
            .version=0x00,
            .command=CMD_SEND_DATA,
            .data_type=DT_UNIT,
            .data_unit=&du
        };
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
    } else if (strcmp(topic, TOPIC_FAN_SET) == 0) {
        uint8_t fan_speed = (uint8_t)ascii_bytes_to_int(payload, length, FAN_LOW);
        DataUnit du;
        DataUnitDTO du_params = {
            .dpid=DPID_STATE,
            .type=TYPE_CHAR,
            .byte_value=fan_speed
        };
        init_data_unit(&du, &du_params);
        DataFrame respFrame;
        DataFrameDTO params = {
            .version=0x00,
            .command=CMD_SEND_DATA,
            .data_type=DT_UNIT,
            .data_unit=&du
        };
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
    }

    LOGLN(topic);
    for (int i=0; i<length; i++) {
        LOG((uint8_t)payload[i]);
        LOG(" ");
    }
    LOGLN("")
    LOGLN(length);
}

void write_data_frame(DataFrame *frame) {
    BytesArray buffer;
    df2bytes(&buffer, frame);
    LOGBUF(buffer, "TX");
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

void send_data(DataFrame *frame)
{
    if (frame->data_type != DT_UNIT) {
        return;
    }

    DataUnit *du = frame->data_unit;
    switch(du->dpid) {
        case DPID_HUMIDITY: {
            char str_value[5];
            sprintf(str_value, "%d", du->int_value);
            logf("Sending humidity: %d", du->int_value);
            mqtt_client.publish("klarta/humidity/get", str_value);
            break;
        }
        case DPID_STATE: {
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            logf("Sending state: %d", du->byte_value);
            mqtt_client.publish("klarta/state/get", str_value);
            break;
        }
        case DPID_FAN: {
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            logf("Sending fan speed: %d", du->byte_value);
            mqtt_client.publish("klarta/fan/get", str_value);
            break;
        }
        case DPID_SLEEP: {
            break;
        }
        case DPID_AUTO_MODE: {
            break;
        }
    }
}

void mcu_handler(HardwareSerial *serial)
{
    static size_t lastState;
    static BytesArray buffer;
    static bool initialized = false;

    bool is_ready;
    while (serial->available()) {
        uint8_t in_byte = serial->read();
        is_ready = parse_byte(&buffer, in_byte);
    }

    if (!is_ready || buffer.len < DF_MIN_SIZE) {
        return;
    }
    LOGBUF(buffer, "RX");

    DataFrame *frame = bytes2df(buffer.bytes, buffer.len);
    if (frame == NULL) {
        return;
    }

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
            // asm volatile ("jmp 0");
            break;
        }

        case CMD_QUERY_DATA: {
            // send status over MQTT
            send_data(frame);
            break;
        }
    }
    free_data_frame(frame);
    memset(&buffer, 0, sizeof(BytesArray));
}


