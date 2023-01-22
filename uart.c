#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"
#include <math.h>
#include "MAVLINK/c_library_v2/common/mavlink.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 128

unsigned char tx_buffer_0[TX_BUFFER_SIZE];
unsigned char rx_buffer_0[RX_BUFFER_SIZE];

unsigned char *com1Rx_buf=rx_buffer_0;											// pointer to the buffer for received characters
volatile int com1Rx_head, com1Rx_tail;								// head and tail of the ring buffer for com2 Rx
unsigned char *com1Tx_buf=tx_buffer_0;;											// pointer to the buffer for transmitted characters
volatile int com1Tx_head, com1Tx_tail;								// head and tail of the ring buffer for com2 Tx

unsigned char tx_buffer_1[TX_BUFFER_SIZE];
unsigned char rx_buffer_1[RX_BUFFER_SIZE];

unsigned char *com2Rx_buf=rx_buffer_1;											// pointer to the buffer for received characters
volatile int com2Rx_head, com2Rx_tail;								// head and tail of the ring buffer for com2 Rx
unsigned char *com2Tx_buf=tx_buffer_1;											// pointer to the buffer for transmitted characters
volatile int com2Tx_head, com2Tx_tail;								// head and tail of the ring buffer for com2 Tx

void on_uart_irq0();
void on_uart_irq1();

/***************************************************************************************************
Initialise the serial function including the interrupts.
****************************************************************************************************/
#define UART_ID  (uart ? uart1: uart0)
void setupuart(int uart, int s2,int parity, int b7, int baud){
	uart_init(UART_ID,baud);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, b7, s2, parity);
    uart_set_fifo_enabled(UART_ID, false);
    int UART_IRQ = (UART_ID == uart0 ? UART0_IRQ : UART1_IRQ);
    if(uart){
		irq_set_exclusive_handler(UART_IRQ, on_uart_irq1);
    	irq_set_enabled(UART_IRQ, true);
	} else {
		irq_set_exclusive_handler(UART_IRQ, on_uart_irq0);
    	irq_set_enabled(UART_IRQ, true);
	}
    uart_set_irq_enables(UART_ID, true, false);
}
/***************************************************************************************************
Add a character to the serial output buffer.
****************************************************************************************************/
unsigned char SerialPutchar(int comnbr, unsigned char c) {
    if(comnbr==0)
        {
        int empty=uart_is_writable(uart0);
		if(com1Tx_tail == ((com1Tx_head + 1) % TX_BUFFER_SIZE)); //wait if buffer full
		com1Tx_buf[com1Tx_head] = c;							// add the char
		com1Tx_head = (com1Tx_head + 1) % TX_BUFFER_SIZE;		   // advance the head of the queue
		if(empty)
            {
	        uart_set_irq_enables(uart0, true, true);
			irq_set_pending(UART0_IRQ);
		    }
        }
        else
        {
        int empty=uart_is_writable(uart1);
		if(com2Tx_tail == ((com2Tx_head + 1) % TX_BUFFER_SIZE)); //wait if buffer full
		com2Tx_buf[com2Tx_head] = c;							// add the char
		com2Tx_head = (com2Tx_head + 1) % TX_BUFFER_SIZE;		   // advance the head of the queue
		if(empty)
            {
	        uart_set_irq_enables(uart1, true, true);
			irq_set_pending(UART1_IRQ);
		    }
        }       
	}

int SerialGetchar(int comnbr) {
	int c;
    if(comnbr==0)
        {
    	c = -1;                                                         // -1 is no data
	    uart_set_irq_enables(uart0, false, true);
		if(com1Rx_head != com1Rx_tail) {                            // if the queue has something in it
			c = com1Rx_buf[com1Rx_tail];                            // get the char
 			com1Rx_tail = (com1Rx_tail + 1) % RX_BUFFER_SIZE;        // and remove from the buffer
		}
		uart_set_irq_enables(uart0, true, true);
        }
        else
        {
      	c = -1;                                                         // -1 is no data
	    uart_set_irq_enables(uart1, false, true);
		if(com2Rx_head != com2Rx_tail) {                            // if the queue has something in it
			c = com2Rx_buf[com2Rx_tail];                            // get the char
 			com2Rx_tail = (com2Rx_tail + 1) % RX_BUFFER_SIZE;        // and remove from the buffer
		}
		uart_set_irq_enables(uart1, true, true);          
        }
	}

void on_uart_irq0() {
    if(uart_is_readable(uart0)) {
		com1Rx_buf[com1Rx_head]  = uart_getc(uart0);   // store the byte in the ring buffer
		com1Rx_head = (com1Rx_head + 1) % RX_BUFFER_SIZE;     // advance the head of the queue
		if(com1Rx_head == com1Rx_tail) {                           // if the buffer has overflowed
			com1Rx_tail = (com1Rx_tail + 1) % RX_BUFFER_SIZE; // throw away the oldest char
		}
    }
    if(uart_is_writable(uart0)){
		if(com1Tx_head != com1Tx_tail) {
			uart_putc_raw(uart0,com1Tx_buf[com1Tx_tail]);
			com1Tx_tail = (com1Tx_tail + 1) % TX_BUFFER_SIZE;       // advance the tail of the queue
		} else {
			uart_set_irq_enables(uart0, true, false);
		}
    }
}
void on_uart_irq1() {
	    while (uart_is_readable(uart1)) {
		com2Rx_buf[com2Rx_head]  = uart_getc(uart1);   // store the byte in the ring buffer
		com2Rx_head = (com2Rx_head + 1) % RX_BUFFER_SIZE;     // advance the head of the queue
		if(com2Rx_head == com2Rx_tail) {                           // if the buffer has overflowed
			com2Rx_tail = (com2Rx_tail + 1) % RX_BUFFER_SIZE; // throw away the oldest char
		}
    }
    if(uart_is_writable(uart1)){
		if(com2Tx_head != com2Tx_tail) {
			uart_putc_raw(uart1,com2Tx_buf[com2Tx_tail]);
			com2Tx_tail = (com2Tx_tail + 1) % TX_BUFFER_SIZE;       // advance the tail of the queue
		} else {
			uart_set_irq_enables(uart1, true, false);
		}
    }
}

