# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_arx_joy_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED arx_joy_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(arx_joy_FOUND FALSE)
  elseif(NOT arx_joy_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(arx_joy_FOUND FALSE)
  endif()
  return()
endif()
set(_arx_joy_CONFIG_INCLUDED TRUE)

# output package information
if(NOT arx_joy_FIND_QUIETLY)
  message(STATUS "Found arx_joy: 0.0.0 (${arx_joy_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'arx_joy' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${arx_joy_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(arx_joy_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${arx_joy_DIR}/${_extra}")
endforeach()
