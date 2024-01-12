#
# Copyright (c) 2020 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# MCUboot related 
# Put the pem certificate under "'your build enviroment'/bootloader/mucboot"
# i.e "/opt/nordic/ncs/v1.9.0/bootloader/mcuboot"
list(APPEND mcuboot_OVERLAY_CONFIG "${CMAKE_CURRENT_SOURCE_DIR}/mcuboot.conf")

set(CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/prj.conf")
# Uncomment the line below when provisioning a device
#list(APPEND CONF_FILE "${CMAKE_CURRENT_SOURCE_DIR}/prj-provisioning.conf")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(levaware_thunder)

# NORDIC SDK APP START
target_sources(app PRIVATE src/main.c)
target_sources(app PRIVATE src/cloud.c)
target_sources(app PRIVATE src/lte_connect_mgr.c)
# NORDIC SDK APP END

zephyr_include_directories(src)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/subsys
    ${CMAKE_CURRENT_SOURCE_DIR}/src/device_config
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/src/power_mgr
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/sensor/bme688
    ${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/sensor/bma253
    ${CMAKE_CURRENT_SOURCE_DIR}/src/apps
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils)

# Application directories
add_subdirectory(src/subsys)
add_subdirectory(src/prod_test)
add_subdirectory(src/device_config)
add_subdirectory(src/sensors)
add_subdirectory(src/power_mgr)
add_subdirectory(src/drivers/sensor/bme688)
add_subdirectory(src/drivers/sensor/bma253)
add_subdirectory(src/apps)
add_subdirectory(src/utils)
