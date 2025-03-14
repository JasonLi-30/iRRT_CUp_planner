#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "moveit_planners_myplanner::moveit_my_planner_interface" for configuration "Debug"
set_property(TARGET moveit_planners_myplanner::moveit_my_planner_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(moveit_planners_myplanner::moveit_my_planner_interface PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmoveit_my_planner_interface.so.2.5.5"
  IMPORTED_SONAME_DEBUG "libmoveit_my_planner_interface.so.2.5.5"
  )

list(APPEND _IMPORT_CHECK_TARGETS moveit_planners_myplanner::moveit_my_planner_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_moveit_planners_myplanner::moveit_my_planner_interface "${_IMPORT_PREFIX}/lib/libmoveit_my_planner_interface.so.2.5.5" )

# Import target "moveit_planners_myplanner::moveit_my_planner_plugin" for configuration "Debug"
set_property(TARGET moveit_planners_myplanner::moveit_my_planner_plugin APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(moveit_planners_myplanner::moveit_my_planner_plugin PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmoveit_my_planner_plugin.so"
  IMPORTED_SONAME_DEBUG "libmoveit_my_planner_plugin.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS moveit_planners_myplanner::moveit_my_planner_plugin )
list(APPEND _IMPORT_CHECK_FILES_FOR_moveit_planners_myplanner::moveit_my_planner_plugin "${_IMPORT_PREFIX}/lib/libmoveit_my_planner_plugin.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
