#ifndef SIMPLETUYA_H
#define SIMPLETUYA_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define DF_MIN_SIZE 7
#define DU_MIN_SIZE 4

#define HEADER_SIZE 2
#define DU_COMMANDS_COUNT 4

#define DATA_BUFFER_SIZE 512


typedef enum {
    TYPE_RAW    = 0x00, // N bytes
    TYPE_BOOL   = 0x01, // 1 byte
    TYPE_INT    = 0x02, // 4 bytes
    TYPE_STR    = 0x03, // N bytes
    TYPE_CHAR   = 0x04, // 1 byte
    TYPE_BITMAP = 0x05  // 1/2/4 bytes
} DataType;

typedef enum {
    // module commands
    CMD_HEALTHCHECK     = 0x00,
    CMD_QUERY_PROD_INFO = 0x01,
    CMD_WORKING_MODE    = 0x02,
    CMD_NETWORK_STATUS  = 0x03,
    CMD_SEND_DATA       = 0x06,

    // MCU commands
    CMD_RESET_MODULE    = 0x04,
    CMD_QUERY_DATA      = 0x07,
} Command;

typedef enum {
    STATE_RESTARTED = 0x00,
    STATE_RUNNING   = 0x01
} LastState;

typedef enum {
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

typedef struct DataUnit {
    uint8_t dpid;
    uint8_t type;
    uint16_t value_len;

    union {
        int int_value;
        // char & bool
        uint8_t byte_value;
        // raw & str & bitmap
        uint8_t *array_value;
    };
} DataUnit;

typedef enum {
    DT_EMPTY = 0,
    DT_UNIT,
    DT_RAW,
} FrameDType;

typedef struct DataFrame {
    uint16_t header;
    uint8_t version;
    uint8_t command;
    uint16_t data_len;
    FrameDType data_type;

    union {
        // TODO: use struct instead of pointers
        DataUnit *data_unit;
        uint8_t *raw_data;
    };

    uint8_t checksum;
} DataFrame;

typedef struct BytesArray {
    size_t len;
    uint8_t bytes[DATA_BUFFER_SIZE];
} BytesArray;

typedef struct DataUnitDTO {
    const uint8_t dpid;
    const uint8_t type;
    union {
        int int_value;
        uint8_t byte_value;
        // TODO: use struct instead of pointers
        BytesArray *array_value;
    };
} DataUnitDTO;

typedef struct DataFrameDTO {
    const uint8_t version;
    const uint8_t command;
    const FrameDType data_type;
    union {
        // TODO: use struct instead of pointers
        DataUnit *data_unit;
        BytesArray *raw_data;
    };
} DataFrameDTO;


void u16_to_bytes(uint8_t *dest, uint16_t value);

uint8_t calculate_bytes_checksum(uint8_t *, size_t);

bool is_frame_valid(DataFrame *);

char *bucket_to_str(BytesArray *);

char *frame_to_str(DataFrame *);

void free_data_frame(DataFrame *);

void free_data_unit(DataUnit *);

void free_bytes_bucket(BytesArray *);

DataFrame *bytes2df(uint8_t *src, size_t len);

DataUnit *bytes2du(uint8_t *src, size_t len);

void du2bytes(uint8_t *dest, const DataUnit *);

void df2bytes(BytesArray *dest, const DataFrame *frame);

void init_data_unit(DataUnit *du, const DataUnitDTO *params);

extern void init_data_frame(DataFrame *frame, const DataFrameDTO *params);


#define bytes_to_decimal(T, bytes)                                              \
  ({                                                                            \
    T value = 0;                                                                \
    size_t j = sizeof(T);                                                       \
    for (size_t i = 0; i < sizeof(T); i++) {                                    \
      value |= (T)bytes[--j] << (8 * i);                                        \
    }                                                                           \
    value;                                                                      \
  })


#define decimal_to_bytes(dest, value)                                           \
({                                                                              \
    uint8_t *le_array = (uint8_t *) &value;                                     \
    int j = 0;                                                                  \
    for (int i=sizeof(value) - 1; i>=0; i--) {                                  \
        dest[j++] = le_array[i];                                                \
    }                                                                           \
})

#endif
