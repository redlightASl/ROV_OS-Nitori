# CMake file
# CMake project
project(algorithm)

# CMake includes
set(ALGORITHM_INCLUDES
    Algorithm.h
)

# CMake sources
set(ALGORITHM_C
    Algorithm.c
)

# Create the lib:algorithm library
add_library(${PROJECT_NAME} STATIC 
    ${ALGORITHM_C}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        ${ALGORITHM_INCLUDES}
)