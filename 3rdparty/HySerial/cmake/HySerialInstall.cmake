include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS HySerial
        EXPORT HySerialTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install custom find modules for dependencies
install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/FindLiburing.cmake
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/FindTlExpected.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}/Modules
)

install(EXPORT HySerialTargets
        FILE HySerialTargets.cmake
        NAMESPACE HySerial::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/HySerialConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY AnyNewerVersion
)
configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/HySerialConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/HySerialConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
install(
        FILES
        "${CMAKE_CURRENT_BINARY_DIR}/HySerialConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/HySerialConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)