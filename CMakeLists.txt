cmake_minimum_required(VERSION 3.10)
project(hjdrive)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

find_package(jsoncpp REQUIRED)
set(CMAKE_CXX_FLAGS -pthread)

include_directories(include)
add_executable(main src/main.cpp src/sqlite3.c)
target_link_libraries(main jsoncpp_lib ${CMAKE_DL_LIBS})





