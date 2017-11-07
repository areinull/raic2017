cmake_minimum_required(VERSION 3.5)
project(AI_strategy)

file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/*.cpp"
                  "${CMAKE_SOURCE_DIR}/model/*.cpp"
                  "${CMAKE_SOURCE_DIR}/csimplesocket/*.cpp")

add_definitions(-std=c++14 -static -fno-optimize-sibling-calls -fno-strict-aliasing -D_LINUX -lm -s -x c++ -O2 -Wall -Wtype-limits -Wno-unknown-pragmas) 

add_executable(AI_strategy ${SOURCES})
