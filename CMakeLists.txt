# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set_property(GLOBAL APPEND PROPERTY HEX_FILES_TO_MERGE
      ${app_provision_hex})

set(DTS_ROOT_BINDINGS "${CMAKE_CURRENT_SOURCE_DIR}")
set(BOARD nrf52_pca10040)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(lora)

include_directories(app PRIVATE inc)
include_directories(app PRIVATE src/lmic)

FILE(GLOB app_sources src/*.c)
FILE(GLOB lmic_sources src/lmic/*.c)

target_sources(app PRIVATE
  ${app_sources}
  ${lmic_sources}
  )
