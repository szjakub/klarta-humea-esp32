#include <SPI.h>
#include <WiFi.h>
#include <klarta.h>
#include <secrets.h>
#include <string.h>
#include "Arduino.h"
#include "simpletuya.h"


HardwareSerial *serial = &Serial1;
IPAddress haServerIP(HA_SERVER_ADDR);
WiFiClient wifiClient;
PubSubClient mqttClient(haServerIP, 1883, mqtt_callback, wifiClient);

void setup()
{
    DEBUG_INIT();

    serial->begin(9600, SERIAL_8N1, RX_1, TX_1);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        DEBUG_PRINTF(".");
        delay(1000);
    }

    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
    {
        mqttSubscribe(&mqttClient);
    }
}

void loop()
{
    checkWiFiConnection();
    checkMQTTConnection();
    mqttClient.loop();

    send_healthcheck();
    mcu_handler(serial);
}


void checkWiFiConnection()
{
    static unsigned long last_wifi_check = 0;
    if (WiFi.status() != WL_CONNECTED)
    {
        unsigned long current_time = millis();
        if (current_time - last_wifi_check >= WIFI_RECONNECTION_INTERVAL)
        {
            DEBUG_PRINTF("Reconnecting to WiFI...\n");
            WiFi.disconnect();
            WiFi.reconnect();
            last_wifi_check = current_time;
        }
    }
}


void checkMQTTConnection()
{
    static unsigned long last_mqtt_check = 0;
    if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED)
    {
        unsigned long current_time = millis();
        if (current_time - last_mqtt_check >= MQTT_RECONNECTION_INTERVAL)
        {
            DEBUG_PRINTF("Reconnecting to MQTT...\n");
            mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD);
            mqttSubscribe(&mqttClient);
            last_mqtt_check = current_time;
        }
    }
}


void mqttSubscribe(PubSubClient *mqtt_client)
{
    mqttClient.subscribe(TOPIC_POWER_STATE_SET);
    mqttClient.subscribe(TOPIC_NIGHT_LIGHT_SET);
    mqttClient.subscribe(TOPIC_SLEEP_MODE_SET);
    mqttClient.subscribe(TOPIC_AUTO_MODE_SET);
    mqttClient.subscribe(TOPIC_FAN_SPEED_SET);
    mqttClient.subscribe(TOPIC_AUTO_MODE_SET);
    mqttClient.subscribe(TOPIC_DESIRED_HUMIDITY_SET);
    mqttClient.subscribe(TOPIC_TIMER_SET);
}


int ascii_bytes_to_int(byte *payload, unsigned int length, int fallback)
{
    if (length > 10)
    {
        DEBUG_PRINTF("Value of length %d is too big\n", length);
        return fallback;
    }
    char buffer[11];

    for (int i = 0; i < length; i++)
    {
        byte value = payload[i];
        if (value < 0x30 || value > 0x39)
        {
            DEBUG_PRINTF("ASCII value of %d is not a number\n", (unsigned char)value);
            return fallback;
        }
        buffer[i] = (char)payload[i];
    }
    buffer[length] = '\0';
    return atoi(buffer);
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
    DEBUG_PRINTF("Received data from mqtt topic %s - ", topic);
    for (int i = 0; i < length; i++)
    {
        DEBUG_PRINTF("0x%02X ", (uint8_t)payload[i]);
    }
    DEBUG_PRINTF("\n");
    if (strcmp(topic, TOPIC_POWER_STATE_SET) == 0)
    {
        uint8_t state = (uint8_t)ascii_bytes_to_int(payload, length, 0);
        DataUnit du;
        DataUnitDTO du_params = {.dpid = DPID_POWER_STATE, .type = TYPE_BOOL, .byte_value = state};
        init_data_unit(&du, &du_params);
        DataFrame respFrame;
        DataFrameDTO params = {
            .version = 0x00, .command = CMD_SEND_DATA, .data_type = DT_UNIT, .data_unit = &du};
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
    }
    else if (strcmp(topic, TOPIC_FAN_SPEED_SET) == 0)
    {
        uint8_t fan_speed = (uint8_t)ascii_bytes_to_int(payload, length, FAN_SPEED_LOW);
        DataUnit du;
        DataUnitDTO du_params = {
            .dpid = DPID_FAN_SPEED, .type = TYPE_CHAR, .byte_value = fan_speed};
        init_data_unit(&du, &du_params);
        DataFrame respFrame;
        DataFrameDTO params = {
            .version = 0x00, .command = CMD_SEND_DATA, .data_type = DT_UNIT, .data_unit = &du};
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
    }
    else if (strcmp(topic, TOPIC_AUTO_MODE_SET) == 0)
    {
        uint8_t auto_mode = (uint8_t)ascii_bytes_to_int(payload, length, STATE_OFF);
        DataUnit du;
        DataUnitDTO du_params = {
            .dpid = DPID_AUTO_MODE, .type = TYPE_BOOL, .byte_value = auto_mode};
        init_data_unit(&du, &du_params);
        DataFrame respFrame;
        DataFrameDTO params = {
            .version = 0x00, .command = CMD_SEND_DATA, .data_type = DT_UNIT, .data_unit = &du};
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
    }
    else if (strcmp(topic, TOPIC_NIGHT_LIGHT_SET) == 0)
    {
        uint8_t nlight_level = (uint8_t)ascii_bytes_to_int(payload, length, NIGHT_LIGHT_OFF);
        DataUnit du;
        DataUnitDTO du_params = {
            .dpid = DPID_NIGHT_LIGHT, .type = TYPE_CHAR, .byte_value = nlight_level};
        init_data_unit(&du, &du_params);
        DataFrame respFrame;
        DataFrameDTO params = {
            .version = 0x00, .command = CMD_SEND_DATA, .data_type = DT_UNIT, .data_unit = &du};
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
    }
    else if (strcmp(topic, TOPIC_SLEEP_MODE_SET) == 0)
    {
        uint8_t sleep_mode = (uint8_t)ascii_bytes_to_int(payload, length, STATE_OFF);
        DataUnit du;
        DataUnitDTO du_params = {
            .dpid = DPID_SLEEP_MODE, .type = TYPE_CHAR, .byte_value = sleep_mode};
        init_data_unit(&du, &du_params);
        DataFrame respFrame;
        DataFrameDTO params = {
            .version = 0x00, .command = CMD_SEND_DATA, .data_type = DT_UNIT, .data_unit = &du};
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
    }
    else if (strcmp(topic, TOPIC_DESIRED_HUMIDITY_SET) == 0)
    {
        uint8_t des_humidity = (uint8_t)ascii_bytes_to_int(payload, length, DES_HUMIDITY_CO);
        DataUnit du;
        DataUnitDTO du_params = {
            .dpid = DPID_DES_HUMIDITY, .type = TYPE_CHAR, .byte_value = des_humidity};
        init_data_unit(&du, &du_params);
        DataFrame respFrame;
        DataFrameDTO params = {
            .version = 0x00, .command = CMD_SEND_DATA, .data_type = DT_UNIT, .data_unit = &du};
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
    }
    else if (strcmp(topic, TOPIC_TIMER_SET) == 0)
    {
        uint8_t timer_value = (uint8_t)ascii_bytes_to_int(payload, length, TIMER_OFF);
        DataUnit du;
        DataUnitDTO du_params = {.dpid = DPID_TIMER, .type = TYPE_CHAR, .byte_value = timer_value};
        init_data_unit(&du, &du_params);
        DataFrame respFrame;
        DataFrameDTO params = {
            .version = 0x00, .command = CMD_SEND_DATA, .data_type = DT_UNIT, .data_unit = &du};
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
    }
}

void write_data_frame(DataFrame *frame)
{
    ByteArray buffer;
    df2bytes(&buffer, frame);
    char *str = bytes_array_to_str(&buffer);
    DEBUG_PRINTF("TX %s\n", str);
    free(str);
    serial->write(buffer.bytes, buffer.len);
}

void send_healthcheck(void)
{
    static unsigned long last_healthcheck = 0;
    unsigned long current_time            = millis();

    if (current_time > last_healthcheck + HEALTHCHECK_INTERVAL)
    {
        DataFrame respFrame;
        DataFrameDTO params = {
            .version   = 0x00,
            .command   = CMD_HEALTHCHECK,
            .data_type = DT_EMPTY,
        };
        init_data_frame(&respFrame, &params);
        write_data_frame(&respFrame);
        last_healthcheck = current_time;
    }
}

void send_data(DataFrame *frame)
{
    if (frame->data_type != DT_UNIT)
    {
        return;
    }

    DataUnit *du = frame->data_unit;
    switch (du->dpid)
    {
        case DPID_POWER_STATE:
        {  // 10 / 0A
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            DEBUG_PRINTF("Sending state: %d\n", du->byte_value);
            mqttClient.publish(TOPIC_POWER_STATE_GET, str_value);
            break;
        }
        case DPID_WATER_LEVEL:
        {  // 101 / 65
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            DEBUG_PRINTF("Sending water level state: %d\n", du->byte_value);
            mqttClient.publish(TOPIC_WATER_LEVEL_GET, str_value);
            break;
        }
        case DPID_NIGHT_LIGHT:
        {  // 102 / 66
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            DEBUG_PRINTF("Sending night light state: %d\n", du->byte_value);
            mqttClient.publish(TOPIC_NIGHT_LIGHT_GET, str_value);
            break;
        }
        case DPID_SLEEP_MODE:
        {  // 103 / 67
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            DEBUG_PRINTF("Sending sleep mode state: %d\n", du->byte_value);
            mqttClient.publish(TOPIC_SLEEP_MODE_GET, str_value);
            break;
        }
        case DPID_AUTO_MODE:
        {  // 104 / 68
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            DEBUG_PRINTF("Sending auto mode state: %d\n", du->byte_value);
            mqttClient.publish(TOPIC_AUTO_MODE_GET, str_value);
            break;
        }
        case DPID_DES_HUMIDITY:
        {  // 105 / 69
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            DEBUG_PRINTF("Sending desired humidity: %d\n", du->byte_value);
            mqttClient.publish(TOPIC_DESIRED_HUMIDITY_GET, str_value);
            break;
        }
        case DPID_FAN_SPEED:
        {  // 106 / 6A
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            DEBUG_PRINTF("Sending fan speed level: %d\n", du->byte_value);
            mqttClient.publish(TOPIC_FAN_SPEED_GET, str_value);
            break;
        }
        case DPID_TIMER:
        {  // 108 / 6C
            char str_value[2];
            sprintf(str_value, "%d", du->byte_value);
            DEBUG_PRINTF("Sending timer setting: %d\n", du->byte_value);
            mqttClient.publish(TOPIC_TIMER_GET, str_value);
            break;
        }
        case DPID_HUMIDITY:
        {  // 109 / 6D
            char str_value[5];
            sprintf(str_value, "%d", du->int_value);
            DEBUG_PRINTF("Sending humidity: %d\n", du->int_value);
            mqttClient.publish(TOPIC_HUMIDITY_GET, str_value);
            break;
        }
    }
}

void mcu_handler(HardwareSerial *serial)
{
    static size_t lastState;
    static ByteArray buffer;
    static bool initialized = false;

    bool is_ready;
    while (serial->available())
    {
        uint8_t in_byte = serial->read();
        is_ready        = parse_byte(&buffer, in_byte);
    }

    if (!is_ready || buffer.len < DF_MIN_SIZE)
    {
        return;
    }
    char *str = bytes_array_to_str(&buffer);
    DEBUG_PRINTF("RX %s\n", str);
    free(str);

    DataFrame *frame = bytes2df(buffer.bytes, buffer.len);
    if (frame == NULL)
    {
        return;
    }

    switch (frame->command)
    {
        // <<--- Heartbeat check (0x00)
        case CMD_HEALTHCHECK:
        {
            if (frame->raw_data[0] == 0x00)
            {
                lastState = STATE_RESTARTED;
                // --->> Query product information (0x01)
                DataFrame respFrame;
                DataFrameDTO params = {
                    .version   = 0x00,
                    .command   = CMD_QUERY_PROD_INFO,
                    .data_type = DT_EMPTY,
                };
                init_data_frame(&respFrame, &params);
                write_data_frame(&respFrame);
            }
            else
            {
                lastState = STATE_RUNNING;
                if (!initialized)
                {
                    initialized = true;
                    DataFrame respFrame;
                    DataFrameDTO params = {
                        .version   = 0x00,
                        .command   = CMD_QUERY_STATUS,
                        .data_type = DT_EMPTY,
                    };
                    init_data_frame(&respFrame, &params);
                    write_data_frame(&respFrame);
                }
            }
            break;
        }
        // <<--- Query product information (0x01)
        case CMD_QUERY_PROD_INFO:
        {
            /*
             * there is no need to handle this response,
             * just send another request from the initialization chain
             */

            // --->> Query Working mode (0x02)
            DataFrame respFrame;
            DataFrameDTO params = {
                .version   = 0x00,
                .command   = CMD_WORKING_MODE,
                .data_type = DT_EMPTY,
            };
            init_data_frame(&respFrame, &params);
            write_data_frame(&respFrame);
            break;
        }
        // <<--- Query working mode (0x02)
        case CMD_WORKING_MODE:
        {
            /*
             * there is no need to handle this response,
             * just send another request from the initialization chain
             */

            // --->> Report network status (0x03)
            DataFrame respFrame;
            ByteArray networkStatusData = {.len = 1, .bytes = {STATUS_5}};
            DataFrameDTO params         = {
                        .version   = 0x00,
                        .command   = CMD_NETWORK_STATUS,
                        .data_type = DT_RAW,
            };
            params.raw_data = &networkStatusData;
            init_data_frame(&respFrame, &params);
            write_data_frame(&respFrame);
            break;
        }
            // <<--- Report network status (0x03)
        case CMD_NETWORK_STATUS:
        {
            /*
             * there is no need to handle this response,
             * just send another request from the initialization chain
             */

            // --->> Heartbeat check after the initialization process
            DataFrame respFrame;
            DataFrameDTO params = {
                .version   = 0x00,
                .command   = CMD_HEALTHCHECK,
                .data_type = DT_EMPTY,
            };
            init_data_frame(&respFrame, &params);
            write_data_frame(&respFrame);
            break;
        }

        case CMD_RESET_MODULE:
        {
            break;
        }

        case CMD_QUERY_DATA:
        {
            // send status over MQTT
            send_data(frame);
            break;
        }
    }
    free_data_frame(frame);
    memset(&buffer, 0, sizeof(ByteArray));
}
