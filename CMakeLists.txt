zephyr_sources(max32664c.c max32664c_worker.c)
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(HR)

target_sources(app PRIVATE src/main.c)
