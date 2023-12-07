# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fishbot_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fishbot_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fishbot_FOUND FALSE)
  elseif(NOT fishbot_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fishbot_FOUND FALSE)
  endif()
  return()
endif()
set(_fishbot_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fishbot_FIND_QUIETLY)
  message(STATUS "Found fishbot: 0.0.0 (${fishbot_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fishbot' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fishbot_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fishbot_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fishbot_DIR}/${_extra}")
endforeach()
