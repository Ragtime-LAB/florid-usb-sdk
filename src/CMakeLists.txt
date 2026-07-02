add_library(florid_usb STATIC
    Arm.cpp
)

set_target_properties(florid_usb PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(florid_usb PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(florid_usb PUBLIC
    usb_protocol
    HySerial
    Threads::Threads
)
