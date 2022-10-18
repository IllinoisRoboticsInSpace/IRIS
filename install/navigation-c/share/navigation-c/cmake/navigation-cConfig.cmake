# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_navigation-c_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED navigation-c_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(navigation-c_FOUND FALSE)
  elseif(NOT navigation-c_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(navigation-c_FOUND FALSE)
  endif()
  return()
endif()
set(_navigation-c_CONFIG_INCLUDED TRUE)

# output package information
if(NOT navigation-c_FIND_QUIETLY)
  message(STATUS "Found navigation-c: 0.0.0 (${navigation-c_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'navigation-c' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${navigation-c_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(navigation-c_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${navigation-c_DIR}/${_extra}")
endforeach()
