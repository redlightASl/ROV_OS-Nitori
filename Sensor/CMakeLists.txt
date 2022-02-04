# CMake file
# CMake project
project(sensor)

# CMake includes
set(SENSOR_INCLUDES
    Sensor.h
)

# CMake sources
set(SENSOR_C
    Sensor.c
)

# Create the lib:sensor library
add_library(${PROJECT_NAME} STATIC 
    ${SENSOR_C}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        ${SENSOR_INCLUDES}
)