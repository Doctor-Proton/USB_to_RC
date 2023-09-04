

typedef struct
{
    char name[32];
    float value;

} parameter_t;

int set_param(const char name[], float value);
float get_param(const char name[]);

#define UPPER_US ((uint32_t)get_param("UPPER_US"))
#define LOWER_US ((uint32_t)get_param("LOWER_US"))

#define HEARTBEAT_INTERVAL (uint32_t)get_param("MAVLINK_HEARTBEAT_INTERVAL")
#define OVERRIDE_INTERVAL (uint32_t)get_param("MAVLINK_RC_OVERRIDE_INTERVAL")

#define MAV_SYS_ID (uint8_t)get_param("MAVLINK_SYS_ID")
#define MAV_COMP_ID (uint8_t)get_param("MAVLINK_COMP_ID")
#define RC_TARGET_SYS_ID (uint8_t)get_param("MAVLINK_TARGET_SYS_ID")
#define RC_TARGET_COMP_ID (uint8_t)get_param("MAVLINK_TARGET_SYS_ID")
#define UART_BAUD (uint32_t)get_param("MAVLINK_BAUD")
#define PPM_INVERT ((uint8_t)get_param("PPM_INVERT")!=0)
#define SBUS_INVERT ((uint8_t)get_param("SBUS_INVERT")!=0)