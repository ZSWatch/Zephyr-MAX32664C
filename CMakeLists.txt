cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(HR)

add_subdirectory(drivers)

target_include_directories(app PRIVATE src/firmware)
target_include_directories(app PRIVATE src/Loader)

target_sources(app PRIVATE src/main.c
                            src/Loader/max32664c_bl.c)
                            
