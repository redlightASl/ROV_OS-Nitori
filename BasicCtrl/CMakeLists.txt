# CMake file
# CMake project
project(basicctrl)

# CMake includes
set(BASICCTRL_INCLUDES
    BasicCtrl.h
)

# CMake sources
set(BASICCTRL_C
    BasicCtrl.c
)

# Create the lib:basicctrl library
add_library(${PROJECT_NAME} STATIC 
    ${BASICCTRL_C}
)

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        ${BASICCTRL_INCLUDES}
)