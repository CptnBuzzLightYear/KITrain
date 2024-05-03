# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_clearance_profile_setter_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED clearance_profile_setter_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(clearance_profile_setter_FOUND FALSE)
  elseif(NOT clearance_profile_setter_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(clearance_profile_setter_FOUND FALSE)
  endif()
  return()
endif()
set(_clearance_profile_setter_CONFIG_INCLUDED TRUE)

# output package information
if(NOT clearance_profile_setter_FIND_QUIETLY)
  message(STATUS "Found clearance_profile_setter: 0.0.0 (${clearance_profile_setter_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'clearance_profile_setter' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${clearance_profile_setter_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(clearance_profile_setter_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${clearance_profile_setter_DIR}/${_extra}")
endforeach()
