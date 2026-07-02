pybind11_add_module(florid_usb_py pybind.cpp)
set_target_properties(florid_usb_py PROPERTIES OUTPUT_NAME florid_usb)

# Respect pip install's output directory; default to source-tree for dev builds
if(CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set_target_properties(florid_usb_py PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
else()
    set_target_properties(florid_usb_py PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif()

target_link_libraries(florid_usb_py PRIVATE florid_usb)
