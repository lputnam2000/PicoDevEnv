cmake_minimum_required(VERSION 3.13)

# Initialize the SDK based on PICO_SDK_PATH
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{FREERTOS_KERNEL_PATH}/FreeRTOS_Kernel_import.cmake)


project(my_project)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable with your source files
add_executable(${PROJECT}
    src/main.cpp
)

# Include directories for your project
target_include_directories(${PROJECT} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
)

# Link libraries for FreeRTOS, lwIP, and Wi-Fi support
target_link_libraries(${PROJECT}
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos  # lwIP with FreeRTOS
    FreeRTOS-Kernel-Heap4              # FreeRTOS
    hardware_pwm                       # For PWM (if needed)
)

# Define NO_SYS=0 (using OS support) and LWIP_TIMEVAL_PRIVATE=0
target_compile_definitions(${PROJECT} PRIVATE
    NO_SYS=0
    LWIP_TIMEVAL_PRIVATE=0
)

# Create output files
pico_add_extra_outputs(${PROJECT})
