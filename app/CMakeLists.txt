cmake_minimum_required(VERSION 3.20.0)

file(GLOB_RECURSE files RELATIVE ${CMAKE_SOURCE_DIR} "patches/zephyr/*.patch")
foreach(file ${files})
    message("Checking patch ${file}")
    # Check if patch is already applied using dry-run
    execute_process(COMMAND
                    patch -p1 -d $ENV{ZEPHYR_BASE} -i ${CMAKE_CURRENT_SOURCE_DIR}/${file} --dry-run --reverse --force
                    RESULT_VARIABLE patch_applied
                    OUTPUT_QUIET
                    ERROR_QUIET)

    if(patch_applied EQUAL 0)
        message("Patch ${file} already applied, skipping")
    else()
        message("Applying patch ${file}")
        execute_process(COMMAND
                        patch -p1 -d $ENV{ZEPHYR_BASE} -i ${CMAKE_CURRENT_SOURCE_DIR}/${file} -r - --no-backup-if-mismatch)
    endif()
endforeach()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(HR)

target_include_directories(app PRIVATE drivers)
target_include_directories(app PRIVATE src/firmware)

add_subdirectory(drivers)

target_sources(app PRIVATE src/main.c)
