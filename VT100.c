  /*
    USB to RC - Convert USB gamepads, joysticks, etc to RC (ppm, sbus, mavlink)
    Copyright (C) 2023  Greg Wood

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
  */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "VT100.h"

#include "output.h"
#include "usb_task.h"
#include "io.h"

uint32_t full_redraw_timer=0;
float change_buffer[128]={0};
float last_value[128]={0};
int last_connected_state=0;
bool return_on_any_key=false;

enum {
    MENU_NORMAL,
    MENU_MAIN,
    //MENU_DESCRIPTOR,
} menu_state=MENU_NORMAL, last_menu_state=MENU_NORMAL;

#define RED_TEXT_STRING "\033[31m"
#define DEFAULT_TEXT_STRING "\033[39m"
#define RED_BACKGROUND_STRING "\033[41m"
#define DEFAULT_BACKGROUND_STRING "\033[49m"
#define CLEAR_SCREEN_STRING "\033[J"
#define NO_USB_STRING "No USB device connected\n"

void red_text(void)
{
    write(1,RED_TEXT_STRING,sizeof(RED_TEXT_STRING));
}

void default_text(void)
{
    write(1,DEFAULT_TEXT_STRING,sizeof(DEFAULT_TEXT_STRING));
}

void red_background(void)
{
    write(1,RED_BACKGROUND_STRING,sizeof(RED_BACKGROUND_STRING));
}

void default_background(void)
{
    write(1,DEFAULT_BACKGROUND_STRING,sizeof(DEFAULT_BACKGROUND_STRING));  
}

void set_cursor(uint16_t x, uint16_t y)
{
char buffer[32];
int count;
count=snprintf(buffer,sizeof(buffer),"\033[%d;%df",y,x);
write(1,buffer,count);
}

void clear_screen(void)
{
    write(1,CLEAR_SCREEN_STRING,sizeof(CLEAR_SCREEN_STRING));
}

void set_terminal(void)
{
    printf("\033[8;%d;%dt",40,160);
}

void print_normal(void)
{
    
    uint16_t vid,pid;
    char print_buffer[64];

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
        write(1,NO_USB_STRING,sizeof(NO_USB_STRING));
        default_text();
        full_redraw_timer=0;
        last_connected_state=usb_connected();
        return;
        }
    last_connected_state=usb_connected();
    if(full_draw==true)
        {
        set_terminal();
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
    default_background();
    for(int i=0;i<count;i++)
    {
        float value;
        char buffer[11];
        get_var(buffer,sizeof(buffer),&value,i);
        float delta=value-last_value[i];
        if(value!=0)
            delta/=value;
            else
            delta=0;
        change_buffer[i]=change_buffer[i]*CHANGE_BUFFER_LPF + (1-CHANGE_BUFFER_LPF)*(delta);

        
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
                int count=snprintf(print_buffer,sizeof(print_buffer),(const char *)printf_modifier,value);
                write(1,print_buffer,count);
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
(void)arg;
vTaskDelay(5000);
gpio_init(TEST_GPIO);
gpio_set_dir(TEST_GPIO,true);
for(;;)
{

    switch(menu_state)
    {
        case MENU_NORMAL:
            if(last_menu_state!=menu_state)
                full_redraw_timer=0;
            last_menu_state=menu_state;
            gpio_put(TEST_GPIO,true);
            print_normal();
            gpio_put(TEST_GPIO,false);
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
