typedef enum
{
    MODE_NO_OUTPUT=0,
    MODE_SBUS_OUT=1,
    MODE_PPM_OUT=2
} OutputType_t;



void ppm_sbus_output_init(OutputType_t Type);
void assign_output_channels(uint16_t Channels[]);
void output_tick(void);
void SendPPM(uint16_t Channels[], uint8_t ChannelCount);
void SendSbus(uint16_t Channels[], uint8_t ChannelCount);
void output_clear(void);
void output_init(void);


#define UART_TX_GPIO 8
#define UART_RX_GPIO 9

#define HEARTBEAT_INTERVAL 1000

//stuff below needs to be configurable

#define RC_CHANNELS_OVERRIDE_INTERVAL 25
#define MAV_SYS_ID 255
#define MAV_COMP_ID 2
#define RC_TARGET_SYS_ID 1
#define RC_TARGET_COMP_ID 1


#define UART_BAUD 115200

#define MAX_OUTPUT_CHANNELS 16

#define UPPER_US 2000
#define LOWER_US 1000

#define UPPER_LSB 1792
#define LOWER_LSB 192