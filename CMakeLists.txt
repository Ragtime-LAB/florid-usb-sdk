cmake_minimum_required(VERSION 3.21)
project(florid_usb_sdk VERSION 0.2.3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_EXAMPLES "Build example programs" ON)
option(BUILD_PYTHON "Build Python bindings (requires numpy)" OFF)

# ── Dependencies ──
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(SdkDependencies)

# ── Protocols (usb_protocol INTERFACE target) ──
set(BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
add_subdirectory(protocols ${CMAKE_CURRENT_BINARY_DIR}/protocols)

# ── Library ──
add_subdirectory(src)

# ── Python bindings ──
if(BUILD_PYTHON)
  find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)
  add_subdirectory(python)
endif()

# ── Examples ──
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
