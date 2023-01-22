/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "hardware/uart.h"
#include "bsp/board.h"
#include "usb_control_decode.h"
#include "FreeRTOS.h"
#include "task.h"
#include "output.h"
#include "usb_task.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

extern void cdc_task(void);
extern void hid_app_task(void);

void led_blinking_task(void *p);
void usb_host_task(void *p);

#define LED_TASK_STACKSIZE 256
StaticTask_t xLEDTaskBuffer;
StackType_t xLEDTaskStack[ LED_TASK_STACKSIZE ];

#define USB_TASK_STACKSIZE 512
StaticTask_t xUSBTaskBuffer;
StackType_t xUSBTaskStack[ USB_TASK_STACKSIZE ];

#define OUTPUT_TASK_STACKSIZE 1024
StaticTask_t xOutputTaskBuffer;
StackType_t xOutputTaskStack[ OUTPUT_TASK_STACKSIZE ];

/*------------- MAIN -------------*/
int main(void)
{
  board_init(); 
  stdio_init_all();

  printf("TinyUSB Host CDC MSC HID Example\r\n");
  
  xTaskCreateStatic(led_blinking_task, "LED_Task", LED_TASK_STACKSIZE, NULL, 1, xLEDTaskStack,&xLEDTaskBuffer);
  xTaskCreateStatic(usb_host_task, "USB_Task", USB_TASK_STACKSIZE, NULL, 255, xUSBTaskStack,&xUSBTaskBuffer);
  xTaskCreateStatic(output_task, "output_Task", OUTPUT_TASK_STACKSIZE, NULL, 250, xOutputTaskStack,&xOutputTaskBuffer);
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
  const uint32_t interval_ms = 1000;
  static uint32_t start_ms = 0;

  static bool led_state = false;

  // Blink every interval ms
  //if ( board_millis() - start_ms < interval_ms) return; // not enough time
  //start_ms += interval_ms;
while (1)
  {
  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
  vTaskDelay(100);
  }
}

#if 0
#define IDLE_TASK_STACKSIZE 256
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ IDLE_TASK_STACKSIZE ];

#define TIMER_TASK_STACKSIZE 256
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ TIMER_TASK_STACKSIZE ];

/* configSUPPORT_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */


    /* Pass out a pointer to the StaticTask_t structure in which the Idle task's
    state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = IDLE_TASK_STACKSIZE;
}
/*-----------------------------------------------------------*/

/* configSUPPORT_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                     StackType_t **ppxTimerTaskStackBuffer,
                                     uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */


    /* Pass out a pointer to the StaticTask_t structure in which the Timer
    task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
    Note that, as the array is necessarily of type StackType_t,
    configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = TIMER_TASK_STACKSIZE;
}

#endif
