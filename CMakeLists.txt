cmake_minimum_required(VERSION 3.20.0)
set(BOARD nucleo_l432kc)

find_package(Zephyr)
project(RBFPID-Balbot)

file(GLOB SRC src/*.cpp)
target_sources(app PRIVATE src/main.c  ${SRC})
set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated/)