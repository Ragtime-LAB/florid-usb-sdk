# ── Third-party libraries (add_subdirectory from 3rdparty/) ──

# unordered_dense (header-only INTERFACE library → target: unordered_dense / unordered_dense::unordered_dense)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/unordered_dense ${CMAKE_CURRENT_BINARY_DIR}/3rdparty/unordered_dense)

# HySerial (serial library → target: HySerial)
set(HS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(HS_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/HySerial ${CMAKE_CURRENT_BINARY_DIR}/3rdparty/HySerial)

# System libraries
find_package(Threads REQUIRED)
