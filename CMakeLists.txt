# Copyright (c) 2025 ZSWatch Project
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SENSOR_MAX32664C)
  zephyr_include_directories(include)
endif()

add_subdirectory_ifdef(CONFIG_SENSOR drivers)
