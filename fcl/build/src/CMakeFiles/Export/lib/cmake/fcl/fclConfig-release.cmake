#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fcl" for configuration "Release"
set_property(TARGET fcl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(fcl PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libfcl.so.0.5.0"
  IMPORTED_SONAME_RELEASE "libfcl.so.7"
  )

list(APPEND _IMPORT_CHECK_TARGETS fcl )
list(APPEND _IMPORT_CHECK_FILES_FOR_fcl "${_IMPORT_PREFIX}/lib/libfcl.so.0.5.0" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
