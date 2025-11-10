# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_bender_manipulation_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED bender_manipulation_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(bender_manipulation_FOUND FALSE)
  elseif(NOT bender_manipulation_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(bender_manipulation_FOUND FALSE)
  endif()
  return()
endif()
set(_bender_manipulation_CONFIG_INCLUDED TRUE)

# output package information
if(NOT bender_manipulation_FIND_QUIETLY)
  message(STATUS "Found bender_manipulation: 0.0.0 (${bender_manipulation_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'bender_manipulation' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT bender_manipulation_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(bender_manipulation_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${bender_manipulation_DIR}/${_extra}")
endforeach()
