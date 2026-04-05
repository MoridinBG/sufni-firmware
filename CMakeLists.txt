# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.21.1)

if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico_w)
    message(STATUS "PICO_BOARD not set, defaulting to pico_w")
endif()

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(sufni-suspension-telemetry)

set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(USB_UART_DEBUG "Enable USB UART for debugging (disables MSC)" OFF)
option(LOG_TO_FILE "Enable logging to file on the SD card" OFF)

pico_sdk_init()

add_executable(sufni-suspension-telemetry
    src/fw/main.c
    src/fw/fw_init.c
    src/fw/data_acquisition.c
    src/fw/data_sync.c
    src/fw/data_storage.c
    src/fw/display.c
    src/fw/helpers.c
    src/fw/sensor_setup.c
    src/fw/state_views.c
    src/fw/calibration_storage.c
    src/fw/calibration_flow.c
    src/fs/microsd.c
    src/net/tcpclient.c
    src/net/tcpserver.c
    src/ntp/ntp.c
    src/pio_i2c/pio_i2c.c
    src/rtc/ds3231.c
    src/ui/pushbutton.c
    src/util/config.c
    src/util/log.c
    src/util/list.c
    src/sensor/travel/rotational.c
    src/sensor/travel/linear.c
    src/sensor/imu/imu_sensor.c
)

if(NOT IMU_FRAME)
    set(IMU_FRAME "NONE")
endif()
if(NOT IMU_FORK)
    set(IMU_FORK "NONE")
endif()
if(NOT IMU_REAR)
    set(IMU_REAR "NONE")
endif()

if(NOT IMU_FRAME STREQUAL "NONE" OR NOT IMU_FORK STREQUAL "NONE" OR NOT IMU_REAR STREQUAL "NONE")
    target_sources(sufni-suspension-telemetry PRIVATE
        src/sensor/imu/lsm6dso.c
        src/sensor/imu/mpu6050.c
    )
endif()

if(NOT USB_UART_DEBUG)
    target_sources(sufni-suspension-telemetry PRIVATE
        src/msc/msc_disk.c
        src/msc/sst_usb_descriptors.c
    )
endif()

pico_generate_pio_header(sufni-suspension-telemetry
    ${CMAKE_CURRENT_LIST_DIR}/src/pio_i2c/i2c.pio
)

add_subdirectory(external/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src firmware/external/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src)
add_subdirectory(external/pico-as5600 firmware/external/pico-as5600)
add_subdirectory(external/pico-ssd1306 firmware/external/pico-ssd1306)

set(NEEDS_SPI OFF)

if(${DISP_PROTO} STREQUAL SPI)
    target_compile_options(sufni-suspension-telemetry PUBLIC
        -DSPI_DISPLAY
    )
    set(NEEDS_SPI ON)
endif()

if(${SPI_MICROSD})
    target_compile_options(sufni-suspension-telemetry PUBLIC
        -DSPI_MICROSD
    )
    set(NEEDS_SPI ON)
endif()

if(${FORK_LINEAR})
    target_compile_options(sufni-suspension-telemetry PUBLIC
        -DFORK_LINEAR
    )
endif()

if(${SHOCK_LINEAR})
    target_compile_options(sufni-suspension-telemetry PUBLIC
        -DSHOCK_LINEAR
    )
endif()

# Frame IMU
if(IMU_FRAME STREQUAL "LSM6DSO")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FRAME=IMU_LSM6DSO)
elseif(IMU_FRAME STREQUAL "MPU6050")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FRAME=IMU_MPU6050)
else()
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FRAME=IMU_NONE)
endif()

if(IMU_FRAME_PROTO STREQUAL "SPI")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FRAME_SPI)
    set(NEEDS_SPI ON)
else()
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FRAME_I2C)
endif()

# Fork IMU
if(IMU_FORK STREQUAL "LSM6DSO")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FORK=IMU_LSM6DSO)
elseif(IMU_FORK STREQUAL "MPU6050")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FORK=IMU_MPU6050)
else()
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FORK=IMU_NONE)
endif()

if(IMU_FORK_PROTO STREQUAL "SPI")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FORK_SPI)
    set(NEEDS_SPI ON)
else()
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_FORK_I2C)
endif()

# Rear IMU
if(IMU_REAR STREQUAL "LSM6DSO")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_REAR=IMU_LSM6DSO)
elseif(IMU_REAR STREQUAL "MPU6050")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_REAR=IMU_MPU6050)
else()
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_REAR=IMU_NONE)
endif()

if(IMU_REAR_PROTO STREQUAL "SPI")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_REAR_SPI)
    set(NEEDS_SPI ON)
else()
    target_compile_definitions(sufni-suspension-telemetry PUBLIC IMU_REAR_I2C)
endif()

if(NEEDS_SPI)
    target_link_libraries(sufni-suspension-telemetry hardware_spi)
endif()

target_include_directories(sufni-suspension-telemetry PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/src/net
)

if(NOT USB_UART_DEBUG)
    target_include_directories(sufni-suspension-telemetry PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/src/msc
    )
endif()

target_link_libraries(sufni-suspension-telemetry
    pico_stdlib
    pico_multicore
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_sntp
    pico_lwip_netif
    pico_lwip_mdns
    pico_aon_timer
    hardware_adc
    hardware_sleep
    tinyusb_board
    hardware_i2c
    hardware_pio
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
    as5600
    ssd1306
)

# Platform-specific libraries
if(PICO_BOARD STREQUAL "pico2_w")
    target_link_libraries(sufni-suspension-telemetry
        hardware_powman
    )
endif()

if(NOT USB_UART_DEBUG)
    target_link_libraries(sufni-suspension-telemetry
        tinyusb_device
    )
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(sufni-suspension-telemetry PUBLIC
        -DSNTP_DEBUG=LWIP_DBG_ON -DUSE_PRINTF=1 -DUSE_DBG_PRINTF=1)
endif()

# GPS Module
if(NOT GPS_MODULE)
    set(GPS_MODULE "NONE")
endif()

if(NOT GPS_MODULE STREQUAL "NONE")
    target_sources(sufni-suspension-telemetry PRIVATE
        src/sensor/gps/gps_sensor.c
        src/sensor/gps/nmea_util.c
        src/sensor/gps/lc76g.c
    )

    set(LWGPS_OPTS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/sensor/gps/lwgps_opts.h)
    add_subdirectory(external/lwgps/lwgps ${CMAKE_CURRENT_BINARY_DIR}/lwgps)
    target_link_libraries(sufni-suspension-telemetry lwgps)
endif()

if(GPS_MODULE STREQUAL "LC76G")
    target_compile_definitions(sufni-suspension-telemetry PUBLIC GPS_MODULE=GPS_LC76G)
else()
    target_compile_definitions(sufni-suspension-telemetry PUBLIC GPS_MODULE=GPS_NONE)
endif()

if(USB_UART_DEBUG)
    target_compile_definitions(sufni-suspension-telemetry PRIVATE
        USB_UART_DEBUG=1
        CFG_TUD_MSC=0
    )
    pico_enable_stdio_usb(sufni-suspension-telemetry 1)
else()
    target_compile_definitions(sufni-suspension-telemetry PRIVATE
        CFG_TUD_MSC=1
    )
endif()

if(LOG_TO_FILE)
    target_compile_definitions(sufni-suspension-telemetry PRIVATE LOG_TO_FILE=1)
endif()

pico_add_extra_outputs(sufni-suspension-telemetry)

# Create config-independent symlinks in build/ root for IDE integration.
# Allows .vscode/c_cpp_properties.json to use fixed paths regardless of preset.
if(NOT CMAKE_BINARY_DIR STREQUAL "${CMAKE_SOURCE_DIR}/build")
    execute_process(COMMAND ${CMAKE_COMMAND} -E rm -f "${CMAKE_SOURCE_DIR}/build/compile_commands.json")
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
        "${CMAKE_BINARY_DIR}/compile_commands.json"
        "${CMAKE_SOURCE_DIR}/build/compile_commands.json"
    )
    execute_process(COMMAND ${CMAKE_COMMAND} -E rm -rf "${CMAKE_SOURCE_DIR}/build/generated")
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
        "${CMAKE_BINARY_DIR}/generated"
        "${CMAKE_SOURCE_DIR}/build/generated"
    )
endif()
