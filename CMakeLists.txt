add_library(kitronik_pico_simply_robotics STATIC
    src/kitronik_simply_robotics.cpp
)

target_include_directories(kitronik_pico_simply_robotics PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(kitronik_pico_simply_robotics
    pico_stdlib
    hardware_pwm
    hardware_pio
)

pico_generate_pio_header(kitronik_pico_simply_robotics
    ${CMAKE_CURRENT_LIST_DIR}/src/servo_pwm.pio
)
