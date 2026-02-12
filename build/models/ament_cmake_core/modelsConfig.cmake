# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_models_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED models_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(models_FOUND FALSE)
  elseif(NOT models_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(models_FOUND FALSE)
  endif()
  return()
endif()
set(_models_CONFIG_INCLUDED TRUE)

# output package information
if(NOT models_FIND_QUIETLY)
  message(STATUS "Found models: 0.0.0 (${models_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'models' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${models_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(models_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${models_DIR}/${_extra}")
endforeach()
