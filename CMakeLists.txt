# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nau7802_driver)

include_directories("../boards/nrf52833dk/BSP"
                    "../lib/Drivers"
                    "../lib/nau7802")

add_compile_definitions(DEBUG)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources}
                ../boards/nrf52833dk/BSP/nrf52833dk_bsp.c
                ../lib/nau7802/nau7802.c)
