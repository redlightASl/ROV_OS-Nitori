# CMake file
# CMake project
project(port)

# Create the lib:acc library
target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${PROJECT_SOURCE_DIR}/Cortex-M4
        ${PROJECT_SOURCE_DIR}
)
