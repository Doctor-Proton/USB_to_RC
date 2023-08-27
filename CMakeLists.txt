# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13) #- commented out for nested cmake
set(PICO_SDK_PATH "/home/greg/pico/pico-sdk") 
set(FREERTOS_KERNEL_PATH "../FreeRTOS-Kernel")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
#set(PICO_SDK_PATH "/home/greg/pico/pico-sdk") - commented out for nested cmake

# Pull in Pico SDK (must be before project)
include(pico_sdk_import.cmake)
# Pull in FreeRTOS
include(FreeRTOS_Kernel_import.cmake)


set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
#include(pico_sdk_import.cmake) - commented out for nested cmake

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(USBHost C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_subdirectory(../FreeRTOS+FAT+CLI build)
add_subdirectory(./Pico-PIO-USB)


set(MAKE_FS_DATA_SCRIPT ${CMAKE_CURRENT_LIST_DIR}/external/makefsdata)

if (NOT EXISTS ${MAKE_FS_DATA_SCRIPT})
        file(DOWNLOAD
                https://raw.githubusercontent.com/krzmaz/lwip/e15654409d14a238aec5ed4bd5516063938c9345/src/apps/http/makefsdata/makefsdata
                ${MAKE_FS_DATA_SCRIPT}
                )
endif()
message("Running makefsdata script")
execute_process(COMMAND
        perl ${MAKE_FS_DATA_SCRIPT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        ECHO_OUTPUT_VARIABLE
        ECHO_ERROR_VARIABLE
        )
file(RENAME fsdata.c my_fsdata.c)

# Add any PICO_CONFIG entries specified in the Advanced settings
add_compile_definitions(PICO_DEFAULT_UART_BAUD_RATE=921600)

add_executable(USBHost main.c hid_app.c usb_control_decode.c HIDParser/HIDParser.c 
    output.c usb_task.c sdcard.c hw_config.c ppm.c uart.c dhcpserver/dhcpserver.c wifi.c
    cgi.cpp ssi.cpp VT100.c parameters.c)

pico_set_program_name(USBHost "USBHost")
pico_set_program_version(USBHost "0.1")

pico_enable_stdio_uart(USBHost 1)
pico_enable_stdio_usb(USBHost 0)

# Add the standard library to the build
target_link_libraries(USBHost
        cyw43_driver_base
        pico_stdlib 
        tinyusb_host 
        tinyusb_board 
        FreeRTOS+FAT+CLI 
        hardware_pwm
        pico_cyw43_arch_lwip_sys_freertos
        #ico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_http)

# Add the standard include files to the build
target_include_directories(USBHost PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required


)
target_include_directories(USBHost PUBLIC 
        include/ 
        ../FreeRTOS+FAT+CLI/Lab-Project-FreeRTOS-FAT/include
        ../FreeRTOS+FAT+CLI/portable/RP2040
        ../FreeRTOS+FAT+CLI/include
        ../FReeRTOS+FAT+CLI/FreeRTOS-Kernel/include
        dhcpserver/
)

add_subdirectory(HIDParser)

# Add any user requested libraries
target_link_libraries(USBHost
pico_pio_usb           
)

IF (NOT DEFINED N_SD_CARDS)
  SET(N_SD_CARDS 1)
  ENDIF()
target_compile_definitions(USBHost PUBLIC DEBUG 
    N_SD_CARDS=${N_SD_CARDS}
    NO_SYS=0)

pico_add_extra_outputs(USBHost)

