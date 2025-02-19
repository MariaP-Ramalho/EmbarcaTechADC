# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================



cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Display C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(display display.c inc/ssd1306.c)

pico_set_program_name(display "display")
pico_set_program_version(display "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(display 1)
pico_enable_stdio_usb(display 1)

# Add the standard library to the build
target_link_libraries(display
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(display PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)
pico_generate_pio_header(display ${CMAKE_CURRENT_LIST_DIR}/display.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)
target_sources(display PRIVATE display.c)

# Add any user requested libraries
target_link_libraries(display 
        hardware_i2c
        hardware_pio
        hardware_irq
        hardware_uart
        hardware_adc
        hardware_pwm
        )

pico_add_extra_outputs(display)

