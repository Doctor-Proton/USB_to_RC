
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ppm.h"
#include <math.h>
#include "MAVLINK/c_library_v2/common/mavlink.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "uart.h"
#include "parameters.h"
#include "io.h"


//#define SBUS_OUT 



#define PPM_BASE_TIME 280

#define PPM_CHANNEL_COUNT 9
#define CHANNEL_COUNT MAX_OUTPUT_CHANNELS

#define SBUS_BYTES 25
#define SBUS_BITS_PER_BYTE 12 //1 start bit + 8 data bits + 1 even parity bit + 2 stop bits
#define SBUS_RMT_SIZE ((SBUS_BITS_PER_BYTE+1)*SBUS_BYTES+1)
#define SBUS_BIT_TIME 10    //10 microseconds for 100000 baud.  requires an RMT time base of 1uS

#define LSB_PER_US (((float)(UPPER_LSB-LOWER_LSB))/((float)(UPPER_US-LOWER_US)))
#define ZERO_LSB_US (LOWER_US-LOWER_LSB/LSB_PER_US)

uint16_t Channels[CHANNEL_COUNT]={0};

bool output_stop=true;

void SendPPM(uint16_t Channels[], uint8_t ChannelCount);
void SendSbus(uint16_t Channels[], uint8_t ChannelCount);

SemaphoreHandle_t ppm_output_mutex = NULL;
StaticSemaphore_t ppm_output_mutex_buffer;
/*
 * Initialize the RMT Tx channel
 */

uint32_t SbusValues[SBUS_RMT_SIZE];
#define PPM_OUTPUT_BUFFER_SIZE (PPM_CHANNEL_COUNT+1) //+1 for trailing pulse
uint32_t PPMOutChannels[PPM_OUTPUT_BUFFER_SIZE]; 

static uint16_t output_channels[MAX_OUTPUT_CHANNELS];

#define OUTPUT_PERIOD 25

uint32_t LastOutputTime=0;

uint32_t HeartbeatTick=0;
uint32_t RCOverrideTick=0;

const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;



void output_send_mavlink(mavlink_message_t *msg)
{

   unsigned char buffer[270];
   int length=mavlink_msg_to_send_buffer(buffer,msg);
   for(int i=0;i<length;i++)
    { 
    SerialPutchar(1,buffer[i]);
    }

}

int pwm_dma_chan;
dma_channel_config pwm_dma_chan_config;

void output_mutex_init(void)
{
    ppm_output_mutex = xSemaphoreCreateMutexStatic( &ppm_output_mutex_buffer );



}

void ppm_sbus_output_init(void)
{

    if(xSemaphoreTake(ppm_output_mutex,200)==pdTRUE)
        {
        gpio_set_function(PPM_OUTPUT_PIN, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(PPM_OUTPUT_PIN);
        uint chan = pwm_gpio_to_channel(PPM_OUTPUT_PIN);
        pwm_set_clkdiv_int_frac(slice_num, 124, 0);
        pwm_set_chan_level(slice_num, chan, PPM_BASE_TIME);

        pwm_set_output_polarity(slice_num,false,PPM_INVERT);


        // Setup DMA channel to drive the PWM
        pwm_dma_chan = dma_claim_unused_channel(true);

        pwm_dma_chan_config = dma_channel_get_default_config(pwm_dma_chan);
        // Transfers 32-bits at a time, increment read address so we pick up a new fade value each
        // time, don't increment writes address so we always transfer to the same PWM register.
        channel_config_set_transfer_data_size(&pwm_dma_chan_config, DMA_SIZE_32);
        channel_config_set_read_increment(&pwm_dma_chan_config, true);
        channel_config_set_write_increment(&pwm_dma_chan_config, false);
        // Transfer when PWM slice that is connected to the LED asks for a new value
        channel_config_set_dreq(&pwm_dma_chan_config, DREQ_PWM_WRAP0 + slice_num);

            output_stop=false;
        xSemaphoreGive(ppm_output_mutex);
        }
    gpio_set_function(UART_TX_GPIO, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_GPIO, GPIO_FUNC_UART);
    setupuart(1,1,UART_PARITY_NONE,8,UART_BAUD);

    gpio_set_function(SBUS_TX_GPIO, GPIO_FUNC_UART);
    setupuart(0,2,UART_PARITY_EVEN,8,100000);   //setup sbus uart for 8E2
    if(!SBUS_INVERT)
        gpio_set_outover(SBUS_TX_GPIO,GPIO_OVERRIDE_INVERT);
    
}

void output_clear(void)
{
    output_stop=true;
    if(xSemaphoreTake(ppm_output_mutex,200)==pdTRUE)
        {
        uint slice_num = pwm_gpio_to_slice_num(PPM_OUTPUT_PIN);
        pwm_set_enabled(slice_num,false);
        pwm_set_counter(slice_num,0);
        xSemaphoreGive(ppm_output_mutex);
        }
}

void assign_output_channels(uint16_t Channels[])
{
    for(int i=0;i<MAX_OUTPUT_CHANNELS;i++)
        {
            if(Channels[i]<800 || Channels[i]>2200)
                Channels[i]=1500;
            output_channels[i]=Channels[i];
        }
}

void output_tick(void)
{
if((xTaskGetTickCount()-LastOutputTime)>OUTPUT_PERIOD && xSemaphoreTake(ppm_output_mutex,2)==pdTRUE)
    {
    LastOutputTime=xTaskGetTickCount();
    if(output_stop==false)
        {
        SendPPM(output_channels,PPM_CHANNEL_COUNT);
        SendSbus(output_channels,MAX_OUTPUT_CHANNELS);
        }
    xSemaphoreGive(ppm_output_mutex);
    }
if((xTaskGetTickCount()-HeartbeatTick)>HEARTBEAT_INTERVAL && HEARTBEAT_INTERVAL!=0)
    {
    HeartbeatTick=xTaskGetTickCount();
    mavlink_message_t Heartbeat;
    mavlink_msg_heartbeat_pack_chan(MAV_SYS_ID,MAV_COMP_ID,0,&Heartbeat,MAV_TYPE_GENERIC,MAV_AUTOPILOT_INVALID,0,0,0);
    output_send_mavlink(&Heartbeat);
    }
if((xTaskGetTickCount()-RCOverrideTick)>OVERRIDE_INTERVAL && output_stop==false)
    {
    RCOverrideTick=xTaskGetTickCount();
    mavlink_message_t Override;
        //need to alter to better suit different channel output count
    mavlink_msg_rc_channels_override_pack_chan(MAV_SYS_ID,MAV_COMP_ID,0,&Override,RC_TARGET_SYS_ID,RC_TARGET_COMP_ID,
        output_channels[0],output_channels[1],output_channels[2],output_channels[3],output_channels[4],output_channels[5],
        output_channels[6],output_channels[7],output_channels[8],output_channels[9],output_channels[10],output_channels[11],
        output_channels[12],output_channels[13],output_channels[14],output_channels[15],UINT16_MAX,UINT16_MAX);
    output_send_mavlink(&Override);
    }
    // Setup the channel and set it going

}

void dma_irh() {
    uint slice_num = pwm_gpio_to_slice_num(PPM_OUTPUT_PIN);
    uint chan = pwm_gpio_to_channel(PPM_OUTPUT_PIN);
    dma_hw->ints1 = (1u << pwm_dma_chan);
    pwm_set_chan_level(slice_num, chan, 0);

    


}

void SendPPM(uint16_t Channels[], uint8_t ChannelCount)
{
if(output_stop==true)
    return;
uint slice_num = pwm_gpio_to_slice_num(PPM_OUTPUT_PIN);
uint chan = pwm_gpio_to_channel(PPM_OUTPUT_PIN);
//#warning setup ppm frame send here
for(int i=0;i<ChannelCount;i++)
    PPMOutChannels[i]=Channels[i];
PPMOutChannels[ChannelCount]=0xFFFF;
pwm_set_enabled(slice_num,false);
pwm_set_counter(slice_num,0);
pwm_set_chan_level(slice_num, chan, PPM_BASE_TIME);
//dma_channel_set_irq1_enabled(pwm_dma_chan, true);
//irq_set_exclusive_handler(DMA_IRQ_1, dma_irh);
//irq_set_enabled(DMA_IRQ_1, true);
dma_channel_abort(pwm_dma_chan);
pwm_set_wrap(slice_num,PPMOutChannels[0]);
dma_channel_configure(
    pwm_dma_chan,
    &pwm_dma_chan_config,
    &pwm_hw->slice[slice_num].top, // Write to PWM counter wrap
    &PPMOutChannels[2], // Read values from fade buffer
    PPM_OUTPUT_BUFFER_SIZE-2, // 
    true // Start immediately.
);

    
    pwm_set_enabled(slice_num, true);
    while(pwm_get_counter(slice_num)<5);
    pwm_set_wrap(slice_num,PPMOutChannels[1]);

}

void SendSbus(uint16_t Channels[], uint8_t ChannelCount)
{
    
    uint8_t SbusBytes[SBUS_BYTES]={0};
    SbusBytes[0]=0x0F;
    uint8_t LowByte=1;
    uint8_t LowBit=0;
    for(int i=0;i<16;i++)
        {
        uint8_t BitsSet=0;
        float ftemp=1500;
        if(i<ChannelCount)
            ftemp=(float)Channels[i];
        ftemp-=LOWER_US;
        ftemp*=LSB_PER_US;
        ftemp+=LOWER_LSB;
        if(ftemp<1)
            ftemp=1;
        if(ftemp>2046)
            ftemp=2046;
        uint16_t Value=(uint16_t)ftemp;
        uint16_t mask=0;
        for(int i=0;i<8-LowBit;i++)
            {
            mask|=(1<<i);
            BitsSet++;
            }
        SbusBytes[LowByte]|=(Value&mask)<<LowBit;   //put in the lower bits
        LowByte++;
        mask=0;
        for(int i=(8-LowBit),j=0;i<11 && j<8;i++,j++)
            {
            mask|=(1<<i);
            BitsSet++;
            }
        SbusBytes[LowByte]|=(Value&mask)>>(8-LowBit);
        if(mask==(0xFF<<3))
            {
            LowByte++;
            LowBit=0;
            }
            else
            {
            if(BitsSet<11)
                {
                LowByte++;
                LowBit=0;
                for(int i=BitsSet;i<11;i++)
                    {
                        SbusBytes[LowByte]|=(Value&(1<<i))>>BitsSet;
                        LowBit++;
                    }
                }
                else
                {
                LowBit=11-(8-LowBit);
                }
            }
        }

for(int i=0;i<SBUS_BYTES;i++)
    {
        SerialPutchar(0,SbusBytes[i]);
    }
}