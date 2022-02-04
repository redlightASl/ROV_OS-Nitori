# CMake file
# CMake project
project(sys)

# CMake includes
set(SYS_INCLUDES
    Queue.h
    Signal.h
    System.h
)

# CMake sources
set(SYS_C
    Queue.c
    Signal.c
    System.c
)

# Create the lib:sys library
add_library(${PROJECT_NAME} STATIC 
    ${SYS_C}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        ${SYS_INCLUDES}
)