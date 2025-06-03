#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "airbot_play::airbot_auto_set_zero" for configuration "Release"
set_property(TARGET airbot_play::airbot_auto_set_zero APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(airbot_play::airbot_auto_set_zero PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/airbot_auto_set_zero"
  )

list(APPEND _IMPORT_CHECK_TARGETS airbot_play::airbot_auto_set_zero )
list(APPEND _IMPORT_CHECK_FILES_FOR_airbot_play::airbot_auto_set_zero "${_IMPORT_PREFIX}/bin/airbot_auto_set_zero" )

# Import target "airbot_play::airbot_read_params" for configuration "Release"
set_property(TARGET airbot_play::airbot_read_params APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(airbot_play::airbot_read_params PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/airbot_read_params"
  )

list(APPEND _IMPORT_CHECK_TARGETS airbot_play::airbot_read_params )
list(APPEND _IMPORT_CHECK_FILES_FOR_airbot_play::airbot_read_params "${_IMPORT_PREFIX}/bin/airbot_read_params" )

# Import target "airbot_play::airbot_set_zero" for configuration "Release"
set_property(TARGET airbot_play::airbot_set_zero APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(airbot_play::airbot_set_zero PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/airbot_set_zero"
  )

list(APPEND _IMPORT_CHECK_TARGETS airbot_play::airbot_set_zero )
list(APPEND _IMPORT_CHECK_FILES_FOR_airbot_play::airbot_set_zero "${_IMPORT_PREFIX}/bin/airbot_set_zero" )

# Import target "airbot_play::airbot_play" for configuration "Release"
set_property(TARGET airbot_play::airbot_play APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(airbot_play::airbot_play PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libairbot_play.so.2.9.0"
  IMPORTED_SONAME_RELEASE "libairbot_play.so.2.9"
  )

list(APPEND _IMPORT_CHECK_TARGETS airbot_play::airbot_play )
list(APPEND _IMPORT_CHECK_FILES_FOR_airbot_play::airbot_play "${_IMPORT_PREFIX}/lib/libairbot_play.so.2.9.0" )

# Import target "airbot_play::kdl_parser" for configuration "Release"
set_property(TARGET airbot_play::kdl_parser APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(airbot_play::kdl_parser PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libkdl_parser.so"
  IMPORTED_SONAME_RELEASE "libkdl_parser.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS airbot_play::kdl_parser )
list(APPEND _IMPORT_CHECK_FILES_FOR_airbot_play::kdl_parser "${_IMPORT_PREFIX}/lib/libkdl_parser.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
