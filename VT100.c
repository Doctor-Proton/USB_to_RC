#include <stdio.h>
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "VT100.h"

#include "output.h"
#include "usb_task.h"

uint32_t full_redraw_timer=0;
float change_buffer[128]={0};
float last_value[128]={0};
int last_connected_state=0;
bool return_on_any_key=false;

enum {
    MENU_NORMAL,
    MENU_MAIN,
    MENU_DESCRIPTOR,
} menu_state=MENU_NORMAL, last_menu_state=MENU_NORMAL;

void red_text(void)
{
    printf("\033[31m");
}

void default_text(void)
{
    printf("\033[39m");
}

void red_background(void)
{
    printf("\033[41m");
}

void default_background(void)
{
  printf("\033[49m");  
}

void set_cursor(uint16_t x, uint16_t y)
{
printf("\033[%d;%df",y,x);  
}

void clear_screen(void)
{
    printf("\033[J");
}

void print_normal(void)
{
    
    uint16_t vid,pid;

    bool full_draw=false;
    if((xTaskGetTickCount()-full_redraw_timer)>FULL_DRAW_TIME || full_redraw_timer==0)
        {
            full_redraw_timer=xTaskGetTickCount();
            full_draw=true;
        }
    if((!usb_connected() && full_draw==true) || (!usb_connected() && last_connected_state))
        {
        set_cursor(0,0);
        //clear_screen();
        default_background();
        red_text();
        printf("No USB device connected\n");
        default_text();
        full_redraw_timer=0;
        last_connected_state=usb_connected();
        return;
        }
    last_connected_state=usb_connected();
    if(full_draw==true)
        {
        set_cursor(0,0);
        clear_screen();
        default_background();
        default_text();
        get_vid_pid(&vid,&pid);
        char buffer[128];
        get_manufacturer_string(buffer,sizeof(buffer));
        printf("%s ",buffer);
        get_product_string(buffer,sizeof(buffer));
        printf("%s\nVID:PID = %04X:%04X",buffer,vid,pid);
        if(strlen(get_mixer_filename())!=0)
            printf("\nusing %s",get_mixer_filename());
            else
            printf("\nno mixer file found");
        set_cursor(0,VAR_PRINT_ROW_OFFSET-1);
        for(int i=0;i<VAR_COLUMN_COUNT*VAR_COLUMN_WIDTH;i++)
            printf("-");
        }
    
    int count=get_vars_count();
    for(int i=0;i<count;i++)
    {
        float value;
        char buffer[11];
        get_var(buffer,sizeof(buffer),&value,i);
        change_buffer[i]=change_buffer[i]*CHANGE_BUFFER_LPF + (1-CHANGE_BUFFER_LPF)*(value-last_value[i]);

        default_background();
        char printf_modifier[]="%+1.2f";
        if(strstr(buffer,"out")!=0)
            strncpy(printf_modifier,"%+1.0f",sizeof(printf_modifier));
        if(full_draw==true)
            {

            set_cursor((i%VAR_COLUMN_COUNT)*VAR_COLUMN_WIDTH,VAR_PRINT_ROW_OFFSET+i/VAR_COLUMN_COUNT);
            printf("%s ",buffer);
            if(fabsf(change_buffer[i])>CHANGE_THRESHOLD)
                red_background();
                else
                default_background();
            set_cursor((i%VAR_COLUMN_COUNT)*VAR_COLUMN_WIDTH+10,VAR_PRINT_ROW_OFFSET+i/VAR_COLUMN_COUNT);
            printf((const char *)printf_modifier,value);
            default_background();
            int pos=(i%VAR_COLUMN_COUNT)*VAR_COLUMN_WIDTH+VAR_COLUMN_WIDTH -3;
            if((i%VAR_COLUMN_COUNT) != (VAR_COLUMN_COUNT-1))
                {
                set_cursor(pos,VAR_PRINT_ROW_OFFSET+i/VAR_COLUMN_COUNT);
                printf("|");
                }

            }
            else
            {
            if(last_value[i]!=value || fabsf(change_buffer[i])>(CHANGE_THRESHOLD/2))
                {
                set_cursor((i%VAR_COLUMN_COUNT)*VAR_COLUMN_WIDTH+10,VAR_PRINT_ROW_OFFSET+i/VAR_COLUMN_COUNT);
                if(fabsf(change_buffer[i])>CHANGE_THRESHOLD)
                    red_background();
                    else
                    default_background();
                printf((const char *)printf_modifier,value);
                default_background();               
                }
            }
        last_value[i]=value;
    }

    if(full_draw==true)
        {
        int rows=count/VAR_COLUMN_COUNT;
        if((count%VAR_COLUMN_COUNT)!=0)
            rows++;
        set_cursor(0,VAR_PRINT_ROW_OFFSET+1+rows);
        printf("Press m or M for menu");
        }
    int c=getchar_timeout_us(0);
    if(c!=PICO_ERROR_TIMEOUT)
        {
            if(c=='m' || c=='M')
                menu_state=MENU_MAIN;
        }
}

void print_menu(bool full_draw)
{
    if(full_draw==true)
        {
        set_cursor(0,0);
        clear_screen();
        default_background();
        default_text();
        printf("0 - return to main display\n");
        printf("1 - reload mixer file\n");
        printf("2 - print descriptors\n");
        return_on_any_key=false;       
        }
    int c=getchar_timeout_us(0);
    if(c!=PICO_ERROR_TIMEOUT)
        {
            if(return_on_any_key==true)
                {
                    menu_state=MENU_NORMAL;
                    
                    return;
                }
            if(c=='0')
                menu_state=MENU_NORMAL;
            if(c=='1')
                {
                    reload_mixers();
                    return_on_any_key=true;
                }
            if(c=='2')
                {
                    print_device_descriptor_vt100();
                    return_on_any_key=true;
                }
        }   
}

void VT100_task(void *arg)
{
for(;;)
{

    switch(menu_state)
    {
        case MENU_NORMAL:
            if(last_menu_state!=menu_state)
                full_redraw_timer=0;
            last_menu_state=menu_state;
            print_normal();
            break;
        case MENU_MAIN:
            if(last_menu_state!=menu_state)
                {
                last_menu_state=menu_state;
                print_menu(true);
                }
                else
                {
                print_menu(false);
                }
            break;
    }
    
    vTaskDelay(100);
}
}
