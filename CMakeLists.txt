# CMake file
cmake_minimum_required(VERSION 3.5)

# CMake project
project(Nitori_RTOS)

add_subdirectory(port)
add_subdirectory(thread)
add_subdirectory(sys)
add_subdirectory(basicctrl)
add_subdirectory(sensor)
add_subdirectory(algorithm)
