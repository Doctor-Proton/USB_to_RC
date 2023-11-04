# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13) #- commented out for nested cmake
set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-FAT-CLI-for-RPi-Pico/FreeRTOS-Kernel")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)
# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)


set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
#include(pico_sdk_import.cmake) - commented out for nested cmake

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.5.1")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.5.1 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(USB_to_RC C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_subdirectory(FreeRTOS-FAT-CLI-for-RPi-Pico/FreeRTOS+FAT+CLI build)
add_subdirectory(Pico-PIO-USB)

# Add any PICO_CONFIG entries specified in the Advanced settings
add_compile_definitions(PICO_DEFAULT_UART_BAUD_RATE=921600)

add_executable(USB_to_RC main.c hid_app.c usb_control_decode.c HIDParser/HIDParser.c 
    output.c usb_task.c sdcard.c hw_config.c ppm.c uart.c 
    VT100.c parameters.c usb_descriptors.c stdio_usb_cdc.c)

pico_set_program_name(USB_to_RC "USB_to_RC")
pico_set_program_version(USB_to_RC "0.1")

pico_enable_stdio_uart(USB_to_RC 0)
pico_enable_stdio_usb(USB_to_RC 0)

# Add the standard library to the build
target_link_libraries(USB_to_RC
        pico_stdlib 
        tinyusb_host
        tinyusb_device
        tinyusb_board 
        FreeRTOS+FAT+CLI 
        hardware_pwm
        )

# Add the standard include files to the build
target_include_directories(USB_to_RC PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)
target_include_directories(USB_to_RC PUBLIC 
        include/ 
        ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-FAT-CLI-for-RPi-Pico/Lab-Project-FreeRTOS-FAT/include
        ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-FAT-CLI-for-RPi-Pico/FreeRTOS+FAT+CLI/portable/RP2040
        ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-FAT-CLI-for-RPi-Pico/FreeRTOS+FAT+CLI/include
        ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-FAT-CLI-for-RPi-Pico/FreeRTOS-Kernel/include
)

add_subdirectory(HIDParser)

# Add any user requested libraries
target_link_libraries(USB_to_RC
pico_pio_usb           
)

IF (NOT DEFINED N_SD_CARDS)
  SET(N_SD_CARDS 1)
  ENDIF()
target_compile_definitions(USB_to_RC PUBLIC DEBUG 
    N_SD_CARDS=${N_SD_CARDS}
    NO_SYS=0)

pico_add_extra_outputs(USB_to_RC)

