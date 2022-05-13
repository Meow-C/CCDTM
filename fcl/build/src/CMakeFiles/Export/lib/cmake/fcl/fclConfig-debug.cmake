#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fcl" for configuration "Debug"
set_property(TARGET fcl APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(fcl PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libfcl.so.0.5.0"
  IMPORTED_SONAME_DEBUG "libfcl.so.7"
  )

list(APPEND _IMPORT_CHECK_TARGETS fcl )
list(APPEND _IMPORT_CHECK_FILES_FOR_fcl "${_IMPORT_PREFIX}/lib/libfcl.so.0.5.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
