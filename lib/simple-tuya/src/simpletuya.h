#ifndef SIMPLETUYA_H_
#define SIMPLETUYA_H_
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define DF_MIN_SIZE 7
#define DU_MIN_SIZE 4

#define HEADER_SIZE 2
#define DU_COMMANDS_COUNT 4

#define DATA_BUFFER_SIZE 512

typedef enum
{
    STATE_HEADER_HIGH,
    STATE_HEADER_LOW,
    STATE_VERSION,
    STATE_COMMAND,
    STATE_DATA_LEN_HIGH,
    STATE_DATA_LEN_LOW,
    STATE_DATA,
    STATE_CHECKSUM
} ParserState;

typedef enum
{
    TYPE_RAW    = 0x00,  // N bytes
    TYPE_BOOL   = 0x01,  // 1 byte
    TYPE_INT    = 0x02,  // 4 bytes
    TYPE_STR    = 0x03,  // N bytes
    TYPE_CHAR   = 0x04,  // 1 byte
    TYPE_BITMAP = 0x05   // 1/2/4 bytes
} DataType;

typedef enum
{
    // module commands
    CMD_HEALTHCHECK     = 0x00,
    CMD_QUERY_PROD_INFO = 0x01,
    CMD_WORKING_MODE    = 0x02,
    CMD_NETWORK_STATUS  = 0x03,
    CMD_SEND_DATA       = 0x06,
    CMD_QUERY_STATUS    = 0x08,

    // MCU commands
    CMD_RESET_MODULE = 0x04,
    CMD_QUERY_DATA   = 0x07,
} Command;

typedef enum
{
    STATE_RESTARTED = 0x00,
    STATE_RUNNING   = 0x01
} DeviceState;

typedef enum
{
    // The SIM card is not connected.
    STATUS_1 = 0x00,

    // The module searches for cellular networks.
    STATUS_2 = 0x01,

    // The module is registered with the cellular network
    // but not connected to the network.
    STATUS_3 = 0x02,

    // The module is connected to the network and gets an IP address.
    STATUS_4 = 0x03,

    // The module has been connected to the cloud.
    STATUS_5 = 0x04,

    // SIM card registration is denied,
    // which might be because it is not subscribed to a cellular service provider.
    STATUS_6 = 0x05,

    // The module is ready for pairing.
    STATUS_7 = 0x06,

    // Unknown status
    STATUS_8 = 0xFF,
} NetworkStatus;

typedef struct
{
    uint8_t dpid;
    uint8_t type;
    uint16_t value_len;

    union
    {
        int int_value;
        // char & bool
        uint8_t byte_value;
        // raw & str & bitmap
        uint8_t *array_value;
    };
} DataUnit;

typedef enum
{
    DT_EMPTY = 0,
    DT_UNIT,
    DT_RAW,
} FrameDType;

typedef struct
{
    uint16_t header;
    uint8_t version;
    uint8_t command;
    uint16_t data_len;
    FrameDType data_type;

    union
    {
        DataUnit *data_unit;
        uint8_t *raw_data;
    };
    uint8_t checksum;
} DataFrame;

typedef struct
{
    size_t len;
    uint8_t bytes[DATA_BUFFER_SIZE];
} ByteArray;

typedef struct
{
    const uint8_t dpid;
    const uint8_t type;
    union
    {
        int int_value;
        uint8_t byte_value;
        ByteArray *array_value;
    };
} DataUnitDTO;

typedef struct
{
    const uint8_t version;
    const uint8_t command;
    const FrameDType data_type;
    union
    {
        DataUnit *data_unit;
        ByteArray *raw_data;
    };
} DataFrameDTO;

bool parse_byte(ByteArray *dest, uint8_t in_byte);
void u16_to_bytes(uint8_t *dest, uint16_t value);
uint8_t calculate_bytes_checksum(uint8_t *bytes, size_t len);
bool is_frame_valid(DataFrame *frame);
char *bytes_array_to_str(ByteArray *array);
char *bucket_to_str(ByteArray *array);
char *frame_to_str(DataFrame *frame);
void free_data_frame(DataFrame *frame);
void free_data_unit(DataUnit *unit);
void free_bytes_bucket(ByteArray *array);
DataFrame *bytes2df(uint8_t *src, size_t len);
DataUnit *bytes2du(uint8_t *src, size_t len);
void du2bytes(uint8_t *dest, const DataUnit *);
void df2bytes(ByteArray *dest, const DataFrame *frame);
void init_data_unit(DataUnit *unit, const DataUnitDTO *params);
void init_data_frame(DataFrame *frame, const DataFrameDTO *params);

#define bytes_to_decimal(T, bytes)             \
    ({                                         \
        T value  = 0;                          \
        size_t j = sizeof(T);                  \
        for (size_t i = 0; i < sizeof(T); i++) \
        {                                      \
            value |= (T)bytes[--j] << (8 * i); \
        }                                      \
        value;                                 \
    })

#define decimal_to_bytes(dest, value)                \
    ({                                               \
        uint8_t *le_array = (uint8_t *)&value;       \
        int j             = 0;                       \
        for (int i = sizeof(value) - 1; i >= 0; i--) \
        {                                            \
            dest[j++] = le_array[i];                 \
        }                                            \
    })

#endif  // SIMPLETUYA_H_
