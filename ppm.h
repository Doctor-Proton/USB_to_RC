typedef enum
{
    MODE_NO_OUTPUT=0,
    MODE_SBUS_OUT=1,
    MODE_PPM_OUT=2
} OutputType_t;



void ppm_sbus_output_init(void);
void assign_output_channels(uint16_t Channels[]);
void output_tick(void);
void SendPPM(uint16_t Channels[], uint8_t ChannelCount);
void SendSbus(uint16_t Channels[], uint8_t ChannelCount);
void output_clear(void);
void output_mutex_init(void);



//stuff below needs to be configurable






#define MAX_OUTPUT_CHANNELS 16

//#define UPPER_US 2000
//#define LOWER_US 1000

#define UPPER_LSB 1792
#define LOWER_LSB 192