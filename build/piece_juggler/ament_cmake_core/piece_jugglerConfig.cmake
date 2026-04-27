# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_piece_juggler_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED piece_juggler_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(piece_juggler_FOUND FALSE)
  elseif(NOT piece_juggler_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(piece_juggler_FOUND FALSE)
  endif()
  return()
endif()
set(_piece_juggler_CONFIG_INCLUDED TRUE)

# output package information
if(NOT piece_juggler_FIND_QUIETLY)
  message(STATUS "Found piece_juggler: 1.0.0 (${piece_juggler_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'piece_juggler' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${piece_juggler_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(piece_juggler_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${piece_juggler_DIR}/${_extra}")
endforeach()
