cmake_minimum_required(VERSION 2.8.11 FATAL_ERROR)

project(hjqtvisual)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUIDE_TYPE "Debug")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

SET(QT5_MODULES Widgets PrintSupport)
FIND_PACKAGE(Qt5 REQUIRED COMPONENTS ${QT5_MODULES})

find_package(jsoncpp REQUIRED)
set(CMAKE_CXX_FLAGS  -pthread)

set(CMAKE_PREFIX_PATH "/home/dian/Downloads/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
# set(TORCH_CXX_FLAGS "-D_GLIBCXX_USE_CXX14_ABI=0")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


include_directories(
    include
    ${TORCH_INCLUDE_DIRS}
	)

add_executable(main
    WIN32
    src/main.cpp
    src/mainwindow.cpp
    src/qcustomplot.cpp
    src/sqlite3.c
    src/hjdata.cpp
    src/mainwindow.ui

)

target_link_libraries(main 
    ${Torch_LIBRARIES}
    ${CODE_LIB_FILES}
    Qt5::Widgets
    Qt5::PrintSupport     
    Qt5::Gui
    jsoncpp_lib
    ${CMAKE_DL_LIBS}
)
# add_definitions(-D _GLIBCXX_USE_CXX11_ABI=0)
# target_link_libraries(main )
# target_link_libraries(main )

add_definitions("-Wall -g")

set_property(TARGET main PROPERTY CXX_STANDARD 14)