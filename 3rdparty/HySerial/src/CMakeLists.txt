add_library(HySerial)

file(GLOB_RECURSE HYSERIAL_SOURCES "${PROJECT_SOURCE_DIR}/src/*.cpp")

# Optional stasis/statistics sources
if (HS_ENABLE_STASIS)
  target_compile_definitions(HySerial PUBLIC HS_ENABLE_STASIS)
endif ()

target_sources(HySerial PRIVATE ${HYSERIAL_SOURCES})
target_include_directories(HySerial PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_link_libraries(HySerial PUBLIC
        tl::expected
        Liburing::Liburing
)
target_compile_features(HySerial PUBLIC cxx_std_20)
