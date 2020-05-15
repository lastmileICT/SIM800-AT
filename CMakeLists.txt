# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(src)

target_sources(app PRIVATE src/SIM800-AT.cpp)
