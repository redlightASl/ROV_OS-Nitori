# CMake file
# CMake project
project(port)
# Create the lib:acc library

set(ACC_C
    ${PROJECT_SOURCE_DIR}/HardwareAccelerate/HardwareAccelerate.c
)

set(ACC_INCLUDES
    ${PROJECT_SOURCE_DIR}/HardwareAccelerate
)

add_library(acc STATIC
    ${ACC_C}
)




