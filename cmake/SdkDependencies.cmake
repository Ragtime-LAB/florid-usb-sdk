# ── Third-party libraries (add_subdirectory from 3rdparty/) ──

# unordered_dense (header-only INTERFACE library → target: unordered_dense / unordered_dense::unordered_dense)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/unordered_dense ${CMAKE_CURRENT_BINARY_DIR}/3rdparty/unordered_dense)

# astrial (cross-platform serial library → target: astrial)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/astrial ${CMAKE_CURRENT_BINARY_DIR}/3rdparty/astrial)

# System libraries
find_package(Threads REQUIRED)
