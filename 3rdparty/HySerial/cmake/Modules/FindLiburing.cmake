find_library(LIBURING_LIBRARY NAMES uring liburing REQUIRED)
find_path(LIBURING_INCLUDE_DIR liburing.h)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Liburing
  REQUIRED_VARS LIBURING_LIBRARY LIBURING_INCLUDE_DIR
)

if(Liburing_FOUND AND NOT TARGET Liburing::Liburing)
  add_library(Liburing::Liburing UNKNOWN IMPORTED)
  set_target_properties(Liburing::Liburing PROPERTIES
    IMPORTED_LOCATION "${LIBURING_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${LIBURING_INCLUDE_DIR}"
  )
endif()
mark_as_advanced(LIBURING_LIBRARY LIBURING_INCLUDE_DIR)
