# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_servoarm_mtc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED servoarm_mtc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(servoarm_mtc_FOUND FALSE)
  elseif(NOT servoarm_mtc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(servoarm_mtc_FOUND FALSE)
  endif()
  return()
endif()
set(_servoarm_mtc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT servoarm_mtc_FIND_QUIETLY)
  message(STATUS "Found servoarm_mtc: 0.0.0 (${servoarm_mtc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'servoarm_mtc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT servoarm_mtc_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(servoarm_mtc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${servoarm_mtc_DIR}/${_extra}")
endforeach()
