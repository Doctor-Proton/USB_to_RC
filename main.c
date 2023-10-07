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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "bsp/board.h"
#include "usb_control_decode.h"
#include "FreeRTOS.h"
#include "task.h"
#include "output.h"
#include "usb_task.h"
#include "VT100.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

extern void cdc_task(void);


void led_blinking_task(void *p);
void usb_host_task(void *p);

#define LED_TASK_STACKSIZE 256
StaticTask_t xLEDTaskBuffer;
StackType_t xLEDTaskStack[ LED_TASK_STACKSIZE ];

#define USB_TASK_STACKSIZE 1024
StaticTask_t xUSBTaskBuffer;
StackType_t xUSBTaskStack[ USB_TASK_STACKSIZE ];

#define OUTPUT_TASK_STACKSIZE 1024
StaticTask_t xOutputTaskBuffer;
StackType_t xOutputTaskStack[ OUTPUT_TASK_STACKSIZE ];


#define VT100_TASK_STACKSIZE 1024
StaticTask_t xVT100TaskBuffer;
StackType_t xVT100TaskStack[ VT100_TASK_STACKSIZE ];

/*------------- MAIN --5-----------*/
int main(void)
{
  board_init(); 
  stdio_init_all();
  
  xTaskHandle LEDTaskHandle=xTaskCreateStatic(led_blinking_task, "LED_Task", LED_TASK_STACKSIZE, NULL, 1, xLEDTaskStack,&xLEDTaskBuffer);
  xTaskHandle USBTaskHandle=xTaskCreateStatic(usb_host_task, "USB_Task", USB_TASK_STACKSIZE, NULL, 255, xUSBTaskStack,&xUSBTaskBuffer);
  xTaskHandle OutputTaskHandle=xTaskCreateStatic(output_task, "output_Task", OUTPUT_TASK_STACKSIZE, NULL, 250, xOutputTaskStack,&xOutputTaskBuffer);
  xTaskHandle VT100TaskHandle=xTaskCreateStatic(VT100_task,"VT100_Task",VT100_TASK_STACKSIZE,NULL,240,xVT100TaskStack,&xVT100TaskBuffer);
  UBaseType_t uxCoreAffinityMask;

  uxCoreAffinityMask=(1<<0);
  vTaskCoreAffinitySet(LEDTaskHandle,uxCoreAffinityMask);
  vTaskCoreAffinitySet(OutputTaskHandle,uxCoreAffinityMask);
  vTaskCoreAffinitySet(VT100TaskHandle,uxCoreAffinityMask);


  uxCoreAffinityMask = (( 1 << 1 ));
  vTaskCoreAffinitySet(USBTaskHandle,uxCoreAffinityMask);
  vTaskStartScheduler();

  //tusb_init();

  while (1)
  {

  }

  return 0;
}




//--------------------------------------------------------------------+
// Blinking Task
//--------------------------------------------------------------------+
void led_blinking_task(void *p)
{
  // Blink every interval ms
  //if ( board_millis() - start_ms < interval_ms) return; // not enough time
  //start_ms += interval_ms;
  const int LED_PIN=25;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
while (1)
  {
  gpio_put(LED_PIN, 0);
  vTaskDelay(100);
  gpio_put(LED_PIN, 1);
  vTaskDelay(100);
  }
}


