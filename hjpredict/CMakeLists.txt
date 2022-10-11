cmake_minimum_required(VERSION 3.0)
project(hjpredict)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FALGS} -g")
set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_PREFIX_PATH "/home/dian/Downloads/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")


add_executable(main src/main.cpp)
target_compile_features(main PUBLIC cxx_range_for)
target_link_libraries(main "${TORCH_LIBRARIES}")

set_property(TARGET main PROPERTY CXX_STANDARD 14)

