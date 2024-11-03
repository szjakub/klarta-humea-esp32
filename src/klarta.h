#define BLUE 255, 255, 200
#define RED 200, 255, 255
#define GREEN 255, 200, 255
#define YELLOW 200, 200, 255

#define HEALTHCHECK_INTERVAL 15000 // ms

#define TOPIC_STATE_SET "klarta/state/set"
#define TOPIC_STATE_GET "klarta/state/get"

#define TOPIC_FAN_SET "klarta/fan/set"
#define TOPIC_FAN_GET "klarta/fan/get"


typedef enum
{
    DPID_HUMIDITY  = 0x6D,
    DPID_STATE     = 0x0A,
    DPID_FAN       = 0x6A,
    DPID_SLEEP     = 0x00,
    DPID_AUTO_MODE = 0x00,
    DPID_DROPLET   = 0x00,
} DPID;


typedef enum
{
    FAN_LOW    = 0x00,
    FAN_MEDIUM = 0x01,
    FAN_HIGH   = 0x02,
    FAN_TURBO  = 0x03
} FanSpeed;


typedef enum
{
    STATE_OFF = 0x00,
    STATE_ON  = 0x01
} DeviceState;

