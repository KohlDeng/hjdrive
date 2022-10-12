cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project(HJVisualV2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)


set(CMAKE_BUIDE_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

SET(QT5_MODULES Widgets Gui Core PrintSupport)
FIND_PACKAGE(Qt5 REQUIRED COMPONENTS ${QT5_MODULES})

find_package(jsoncpp REQUIRED)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -pthread")

set(CMAKE_PREFIX_PATH "/home/dian/Downloads/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)

include_directories(
    #include
    ${TORCH_INCLUDE_DIRS}
	)

add_executable(main

    src/main.cpp
    src/mainwindow.cpp
    src/qcustomplot.cpp
    src/sqlite3.c
    src/hjdata.cpp
    src/mainwindow.ui

)

target_link_libraries(main 

    Qt5::Core
    Qt5::Widgets
    Qt5::PrintSupport     
    Qt5::Gui

    jsoncpp_lib
    ${CMAKE_DL_LIBS}

)

target_link_libraries(main ${TORCH_LIBRARIES}) #main reason 

set_property(TARGET main PROPERTY CXX_STANDARD 14)

message("-----------------------")
message("cmake flag is :" ${CMAKE_CXX_FLAGS})
message("torch dir is located at:" ${TORCH_INCLUDE_DIRS})
message("-----------------------")
