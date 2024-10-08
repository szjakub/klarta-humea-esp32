#include "Arduino.h"
#include "HardwareSerial.h"
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <cstdint>
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

void callback(char *topic, byte *payload, unsigned int lenght);
void send_data(DataFrame *frame);
void send_healthcheck(void);
void mcu_handler(HardwareSerial *serial);
void write_data_frame(DataFrame *frame);
bool serial_read_frame(HardwareSerial *serial, BytesArray *buffer);

HardwareSerial *serial = &Serial1;

IPAddress server(192, 168, 1, 21);
WiFiClient wifi_client;
PubSubClient mqtt_client(server, 1883, callback, wifi_client);

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    WiFi.setHostname(HOSTNAME);
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        LOG('.');
    }

    mqtt_client.setClient(wifi_client);
    if(mqtt_client.connect("arduinoClient", "klarta", "password1")) {
        mqtt_client.subscribe("klarta/humidity/set");
        mqtt_client.subscribe("klarta/fan-speed/set");
        mqtt_client.subscribe("klarta/state/set");
        mqtt_client.subscribe("klarta/auto/set");
        mqtt_client.subscribe("klarta/night-mode/set");
    }
    serial->begin(9600, SERIAL_8N1, RX_1, TX_1);
}

void loop() {
    send_healthcheck();
    mcu_handler(serial);
}


void callback(char* topic, byte *payload, unsigned int lenght) {
    Serial.println(topic);
    Serial.println(lenght);
    for (int i=0; i<lenght; i++) {
        Serial.print(*payload++, HEX);
        Serial.print(" ");
    }
    int h = (payload[0] - '0') * 10 + payload[1] - '0';
    Serial.println(h);

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
            LOG("Sending humidity: ");
            LOGLN(du->int_value);
            char str_value[4];
            sprintf(str_value, "%d", du->int_value);
            mqtt_client.publish("klarta/humidity/get", str_value);
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


