# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_position_corrector_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED position_corrector_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(position_corrector_FOUND FALSE)
  elseif(NOT position_corrector_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(position_corrector_FOUND FALSE)
  endif()
  return()
endif()
set(_position_corrector_CONFIG_INCLUDED TRUE)

# output package information
if(NOT position_corrector_FIND_QUIETLY)
  message(STATUS "Found position_corrector: 0.0.0 (${position_corrector_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'position_corrector' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${position_corrector_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(position_corrector_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${position_corrector_DIR}/${_extra}")
endforeach()
