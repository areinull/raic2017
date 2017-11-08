cmake_minimum_required(VERSION 3.5)
project(MyStrategy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -static \
    -fno-optimize-sibling-calls -fno-strict-aliasing -D_LINUX \
    -lm -g -ggdb -Wall -Wextra -pedantic -Wtype-limits -Wno-unknown-pragmas")

file(GLOB strategy_SRC "*.cpp" "*.h" "model/*.cpp" "model/*.h" "csimplesocket/*.cpp" "csimplesocket/*.h")

add_executable(MyStrategy ${strategy_SRC})
