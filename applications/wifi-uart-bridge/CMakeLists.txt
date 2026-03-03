cmake_minimum_required(VERSION 3.20.0)

# Set ZEPHYR_BASE to help VS Code find the package if environment is not yet set
set(ZEPHYR_BASE $ENV{ZEPHYR_BASE} CACHE PATH "Zephyr base")
if(NOT ZEPHYR_BASE)
    set(ZEPHYR_BASE "/home/arrel/gits/zephyr-projects/zephyr")
endif()

# Explicitly use the venv's Python for Zephyr's scripts
set(PYTHON_EXECUTABLE "/home/arrel/gits/zephyr-projects/.venv/bin/python3" CACHE FILEPATH "Python executable")

# Set BOARD before find_package(Zephyr)
set(BOARD esp32c3_supermini CACHE STRING "Selected board")

# Help Zephyr find the Espressif HAL module if it's not being auto-discovered
# by the IDE's CMake run.
set(ZEPHYR_EXTRA_MODULES "/home/arrel/gits/zephyr-projects/modules/hal/espressif" CACHE PATH "Espressif HAL module")

find_package(Zephyr REQUIRED HINTS ${ZEPHYR_BASE})
project(wifi_uart_bridge)

target_sources(app PRIVATE src/main.c src/led.c)
