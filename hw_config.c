/* hw_config.c
Copyright 2021 Carl John Kugler III

Licensed under the Apache License, Version 2.0 (the License); you may not use
this file except in compliance with the License. You may obtain a copy of the
License at

   http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software distributed
under the License is distributed on an AS IS BASIS, WITHOUT WARRANTIES OR
CONDITIONS OF ANY KIND, either express or implied. See the License for the
specific language governing permissions and limitations under the License.
*/
/*

This file should be tailored to match the hardware design.

There should be one element of the spi[] array for each hardware SPI used.

There should be one element of the sd_cards[] array for each SD card slot.
The name is arbitrary. The rest of the constants will depend on the type of
socket, which SPI it is driven by, and how it is wired.

*/

#include <string.h>
//
#include "hw_config.h"
#include "io.h"

void spi1_dma_isr();

// Hardware Configuration of SPI "objects"
// Note: multiple SD cards can be driven by one SPI if they use different slave
// selects.
static spi_t spis[] = {  // One for each SPI.
    {.hw_inst = spi1,    // SPI component
     .miso_gpio = SD_MISO,
     .mosi_gpio = SD_MOSI,
     .sck_gpio = SD_SCK,
     .baud_rate = 12500 * 1000,
     .dma_isr = spi1_dma_isr}};

// Hardware Configuration of the SD Card "objects"
static sd_card_t sd_cards[] = {  // One for each SD card
    {.pcName = "sd0",            // Name used to mount device
     .spi = &spis[0],            // Pointer to the SPI driving this card
     .ss_gpio = SD_CS,              // The SPI slave select GPIO for this SD card
     .use_card_detect = false,
     .card_detect_gpio = 22,   // Card detect
     .card_detected_true = -1,  // What the GPIO read returns when a card is
                               // present. Use -1 if there is no card detect.
     .m_Status = STA_NOINIT}};

void spi1_dma_isr() { spi_irq_handler(&spis[0]); }

/* ********************************************************************** */
size_t sd_get_num() { return count_of(sd_cards); }
sd_card_t *sd_get_by_num(size_t num) {
    if (num <= sd_get_num()) {
        return &sd_cards[num];
    } else {
        return NULL;
    }
}
sd_card_t *sd_get_by_name(const char *const name) {
    size_t i;
    for (i = 0; i < sd_get_num(); ++i) {
        if (0 == strcmp(sd_cards[i].pcName, name)) return &sd_cards[i];
    }
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
size_t spi_get_num() { return count_of(spis); }
spi_t *spi_get_by_num(size_t num) {
    if (num <= sd_get_num()) {
        return &spis[num];
    } else {
        return NULL;
    }
}
/* [] END OF FILE */
