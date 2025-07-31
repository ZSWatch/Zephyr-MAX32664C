cmake_minimum_required(VERSION 3.20.0)

file(GLOB_RECURSE files RELATIVE ${CMAKE_SOURCE_DIR} "patches/zephyr/*.patch")
foreach(file ${files})
    message("Execute patch ${file}")
    execute_process(COMMAND
                    patch -p1 -d $ENV{ZEPHYR_BASE} -i ${CMAKE_CURRENT_SOURCE_DIR}/${file} -r - --no-backup-if-mismatch)
endforeach()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(HR)

target_include_directories(app PRIVATE src/firmware)

target_sources(app PRIVATE src/main.c)
