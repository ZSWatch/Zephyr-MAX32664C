# Copyright (c) 2025 ZSWatch Project
# SPDX-License-Identifier: Apache-2.0

# This file is processed by Zephyr as a module CMakeLists.txt
# It should only define library sources, not be a standalone project

if(CONFIG_SENSOR_MAX32664C)
  zephyr_include_directories(include)
endif()

add_subdirectory_ifdef(CONFIG_SENSOR drivers)
