# CMake file
# CMake project
project(thread)

# CMake includes
set(THREAD_INCLUDES
    Thread/Thread.h
    Thread/Interrupt.h
    Thread/Core.h
)

# CMake sources
set(THREAD_C
    Thread.c
    Core.c
    Interrupt.c
)

# Create the lib:thread library
add_library(${PROJECT_NAME} STATIC 
    ${THREAD_C}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        ${THREAD_INCLUDES}
)