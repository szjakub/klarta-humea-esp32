#define BLUE 255, 255, 200
#define RED 200, 255, 255
#define GREEN 255, 200, 255
#define YELLOW 200, 200, 255

#define HEALTHCHECK_INTERVAL 15000 // ms

#define TOPIC_POWER_STATE_SET "klarta/state/set"
#define TOPIC_POWER_STATE_GET "klarta/state/get"

#define TOPIC_WATER_LEVEL_GET "klarta/wtrlvl/get"

#define TOPIC_NIGHT_LIGHT_SET "klarta/nlight/set"
#define TOPIC_NIGHT_LIGHT_GET "klarta/nlight/get"

#define TOPIC_SLEEP_MODE_SET "klarta/sleep/set"
#define TOPIC_SLEEP_MODE_GET "klarta/sleep/get"

#define TOPIC_AUTO_MODE_SET "klarta/auto/set"
#define TOPIC_AUTO_MODE_GET "klarta/auto/get"

#define TOPIC_DESIRED_HUMIDITY_SET "klarta/deshum/set"
#define TOPIC_DESIRED_HUMIDITY_GET "klarta/deshum/get"

#define TOPIC_FAN_SPEED_SET "klarta/fan/set"
#define TOPIC_FAN_SPEED_GET "klarta/fan/get"

#define TOPIC_TIMER_SET "klarta/timer/set"
#define TOPIC_TIMER_GET "klarta/timer/get"

#define TOPIC_HUMIDITY_GET "klarta/humidity/get"



/*
 * |    DPID    |
 * | Dec | Hex  | Type | Allowed Values |    Description    |
 * |-----|------|------|----------------|-------------------|
 * | 10  | 0x0A | bool | 0/1            | Power State       | OK
 * | 101 | 0x65 | char | 0/1            | Water Level       | OK
 * | 102 | 0x66 | char | 0/1/2/3        | Night Light       | OK
 * | 103 | 0x67 | bool | 0/1            | Sleep Mode        | ?
 * | 104 | 0x68 | bool | 0/1            | Auto Mode         | OK
 * | 105 | 0x69 | char | 0/1/.../8      | Desired Humidity  | ?
 * | 106 | 0x6A | int  | 0/1/2/3        | Fan Speed         | OK
 * | 107 | 0x6B | char | 0/1            | Filter Status     | ?
 * | 108 | 0x6C | char | 0/1/.../12     | Timer Settings    | ?
 * | 109 | 0x6D | int  | 0-100          | Humidity          | OK
 */


typedef enum
{
    DPID_POWER_STATE    = 0x0A,
    DPID_WATER_LEVEL    = 0x65,
    DPID_NIGHT_LIGHT    = 0x66,
    DPID_SLEEP_MODE     = 0x67,
    DPID_AUTO_MODE      = 0x68,
    DPID_DES_HUMIDITY   = 0x69,
    DPID_FAN_SPEED      = 0x6A,
    DPID_FILTER_STATUS  = 0x6B,
    DPID_TIMER          = 0x6C,
    DPID_HUMIDITY       = 0x6D
} DPID;

typedef enum
{
    FAN_SPEED_LOW    = 0x00,
    FAN_SPEED_MEDIUM = 0x01,
    FAN_SPEED_HIGH   = 0x02,
    FAN_SPEED_TURBO  = 0x03
} FanSpeed;


typedef enum
{
    AUTO_MODE_OFF = 0x00,
    AUTO_MODE_ON  = 0x01
} AutoMode;


typedef enum
{
    STATE_OFF = 0x00,
    STATE_ON  = 0x01
} BinaryState;


typedef enum
{
    NIGHT_LIGHT_OFF     = 0x00,
    NIGHT_LIGHT_LOW     = 0x01,
    NIGHT_LIGHT_MED     = 0x02,
    NIGHT_LIGHT_HIGH    = 0x03
} NightLightLevel;


typedef enum
{
    DES_HUMIDITY_CO = 0x00, // continuously
    DES_HUMIDITY_40 = 0x01,
    DES_HUMIDITY_44 = 0x02,
    DES_HUMIDITY_50 = 0x03,
    DES_HUMIDITY_55 = 0x04,
    DES_HUMIDITY_60 = 0x05,
    DES_HUMIDITY_65 = 0x06,
    DES_HUMIDITY_70 = 0x07,
    DES_HUMIDITY_75 = 0x08
} DesiredHumidity;

typedef enum
{
    TIMER_OFF = 0x00,
    TIMER_01H = 0x01,
    TIMER_02H = 0x02,
    TIMER_03H = 0x03,
    TIMER_04H = 0x04,
    TIMER_05H = 0x05,
    TIMER_06H = 0x06,
    TIMER_07H = 0x07,
    TIMER_08H = 0x08,
    TIMER_09H = 0x09,
    TIMER_10H = 0x0A,
    TIMER_11H = 0x0B,
    TIMER_12H = 0x0C,
} TimerSetting;

