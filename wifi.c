#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/fs.h"

#include "lwipopts.h"
#include "cgi.h"
#include "ssi.h"
#include "dhcpserver.h"
#include "output.h"

struct fs_file *vars_file=0;

int fs_open_custom(struct fs_file *file, const char *name)
{
    if(strncmp(name,"/vars.bin",strlen("/vars.bin"))==0)
        {
            file->data=0;
            file->len=get_vars_size();
            file->index=0;
            vars_file=file;
            return 1;
        }
return 0;
}


int fs_read_custom(struct fs_file *file, char *buffer, int count)
{
if(file==vars_file)
    {
    int bytes_read=read_vars_bytes(buffer,file->index,count);
    file->index+=bytes_read;
    if(file->index>file->len)

        return FS_READ_EOF;
        else
        return bytes_read;
    }
return FS_READ_EOF;
}

void fs_close_custom(struct fs_file *file)
{
    if(file==vars_file)
        vars_file=0;
}

void run_server() {
    httpd_init();
    ssi_init();
    cgi_init();
    printf("Http server initialized.\n");
    // infinite loop for now
    for (;;) 
    {
        cyw43_arch_poll();
        vTaskDelay(1);
    }
}

int wifi_core=-1;
void wifi_task(void *arg)
{

    wifi_core=get_core_num();
    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return;
    }
    //cyw43_arch_enable_sta_mode();
    cyw43_arch_enable_ap_mode("picow","raspberry",CYW43_AUTH_WPA2_AES_PSK);
    ip4_addr_t gw, mask;
    IP4_ADDR(&gw, 192, 168, 4, 1);
    IP4_ADDR(&mask, 255, 255, 255, 0);

    // Start the dhcp server
    dhcp_server_t dhcp_server;
    dhcp_server_init(&dhcp_server, &gw, &mask);
    // this seems to be the best be can do using the predefined `cyw43_pm_value` macro:
    // cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
    // however it doesn't use the `CYW43_NO_POWERSAVE_MODE` value, so we do this instead:
    cyw43_wifi_pm(&cyw43_state, cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20, 1, 1, 1));

    /*printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("SHAW-BIB", "HotDogWithMustard", CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");*/

        extern cyw43_t cyw43_state;
        auto ip_addr = cyw43_state.netif[CYW43_ITF_STA].ip_addr.addr;
        printf("IP Address: %lu.%lu.%lu.%lu\n", ip_addr & 0xFF, (ip_addr >> 8) & 0xFF, (ip_addr >> 16) & 0xFF, ip_addr >> 24);
    //}
    // turn on LED to signal connected
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);


    run_server();

    return;
}