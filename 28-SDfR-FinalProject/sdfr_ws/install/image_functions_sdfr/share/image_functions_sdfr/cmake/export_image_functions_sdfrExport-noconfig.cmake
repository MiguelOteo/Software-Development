#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "image_functions_sdfr::image_functions" for configuration ""
set_property(TARGET image_functions_sdfr::image_functions APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(image_functions_sdfr::image_functions PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libimage_functions.so"
  IMPORTED_SONAME_NOCONFIG "libimage_functions.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS image_functions_sdfr::image_functions )
list(APPEND _IMPORT_CHECK_FILES_FOR_image_functions_sdfr::image_functions "${_IMPORT_PREFIX}/lib/libimage_functions.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
