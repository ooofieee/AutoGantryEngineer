#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rm_serial_driver::rm_serial_driver" for configuration ""
set_property(TARGET rm_serial_driver::rm_serial_driver APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rm_serial_driver::rm_serial_driver PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librm_serial_driver.so"
  IMPORTED_SONAME_NOCONFIG "librm_serial_driver.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rm_serial_driver::rm_serial_driver )
list(APPEND _IMPORT_CHECK_FILES_FOR_rm_serial_driver::rm_serial_driver "${_IMPORT_PREFIX}/lib/librm_serial_driver.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
