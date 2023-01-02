# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2021, Nordic Semiconductor ASA

# Zephyr build system configuration files.
#
# Locate the Kconfig and DT config files that are to be used.
# Also, locate the appropriate application config directory.
#
# Outcome:
# The following variables will be defined when this CMake module completes:
#
# - CONF_FILE:              List of Kconfig fragments
# - DTC_OVERLAY_FILE:       List of devicetree overlay files
# - APPLICATION_CONFIG_DIR: Root folder for application configuration
#
# If any of the above variables are already set when this CMake module is
# loaded, then no changes to the variable will happen.
#
# Variables set by this module and not mentioned above are considered internal
# use only and may be removed, renamed, or re-purposed without prior notice.

include_guard(GLOBAL)

include(extensions)

if(DEFINED APPLICATION_CONFIG_DIR)
  string(CONFIGURE ${APPLICATION_CONFIG_DIR} APPLICATION_CONFIG_DIR)
  if(NOT IS_ABSOLUTE ${APPLICATION_CONFIG_DIR})
    get_filename_component(APPLICATION_CONFIG_DIR ${APPLICATION_CONFIG_DIR} ABSOLUTE)
  endif()
else()
  # Application config dir is not set, so we default to the  application
  # source directory as configuration directory.
  set(APPLICATION_CONFIG_DIR ${APPLICATION_SOURCE_DIR})
endif()

zephyr_get(CONF_FILE SYSBUILD LOCAL)
if(DEFINED CONF_FILE)
  # This ensures that CACHE{CONF_FILE} will be set correctly to current scope
  # variable CONF_FILE. An already current scope variable will stay the same.
  set(CONF_FILE ${CONF_FILE})

  # CONF_FILE has either been specified on the cmake CLI or is already
  # in the CMakeCache.txt. This has precedence over the environment
  # variable CONF_FILE and the default prj.conf

  # In order to support a `prj_<name>.conf pattern for auto inclusion of board
  # overlays, then we must first ensure only a single conf file is provided.
  string(REPLACE " " ";" CONF_FILE_AS_LIST "${CONF_FILE}")
  list(LENGTH CONF_FILE_AS_LIST CONF_FILE_LENGTH)
  if(${CONF_FILE_LENGTH} EQUAL 1)
    # Need the file name to look for match.
    # Need path in order to check if it is absolute.
    get_filename_component(CONF_FILE_NAME ${CONF_FILE} NAME)
    if(${CONF_FILE_NAME} MATCHES "prj_(.*).conf")
      set(CONF_FILE_BUILD_TYPE ${CMAKE_MATCH_1})
      set(CONF_FILE_INCLUDE_FRAGMENTS true)
    endif()
  endif()
elseif(CACHED_CONF_FILE)
  # Cached conf file is present.
  # That value has precedence over anything else than a new
  # `cmake -DCONF_FILE=<file>` invocation.
  set(CONF_FILE ${CACHED_CONF_FILE})
elseif(EXISTS   ${APPLICATION_CONFIG_DIR}/prj_${BOARD}.conf)
  set(CONF_FILE ${APPLICATION_CONFIG_DIR}/prj_${BOARD}.conf)

elseif(EXISTS   ${APPLICATION_CONFIG_DIR}/prj.conf)
  set(CONF_FILE ${APPLICATION_CONFIG_DIR}/prj.conf)
  set(CONF_FILE_INCLUDE_FRAGMENTS true)
endif()

# Append any .conf files from snippets
if(DEFINED SNIPPETS)
  message(STATUS "Snippets: ${SNIPPETS}")
  # Convert from space-separated snippets into snippet list
  string(REPLACE " " ";" SNIPPETS_RAW_LIST "${SNIPPETS}")
  # Search for (optional) ${SNIPPET}.conf files
  foreach(SNIPPET IN LISTS SNIPPETS_RAW_LIST)
    # Application level board snippets have precedence
    if(EXISTS     ${APPLICATION_SOURCE_DIR}/${BOARD}/snippets/${SNIPPET}.conf)
      list(APPEND CONF_FILE " ${APPLICATION_SOURCE_DIR}/boards/${BOARD}/snippets/${SNIPPET}.conf")
      set(CONF_FILE_INCLUDE_FRAGMENTS true)
    elseif(EXISTS ${BOARD_DIR}/snippets/${SNIPPET}.conf)
      list(APPEND CONF_FILE " ${BOARD_DIR}/snippets/${SNIPPET}.conf")
      set(CONF_FILE_INCLUDE_FRAGMENTS true)
    endif()
  endforeach()
endif()

if(CONF_FILE_INCLUDE_FRAGMENTS)
  zephyr_file(CONF_FILES ${APPLICATION_CONFIG_DIR}/boards KCONF CONF_FILE BUILD ${CONF_FILE_BUILD_TYPE})
endif()

set(APPLICATION_CONFIG_DIR ${APPLICATION_CONFIG_DIR} CACHE INTERNAL "The application configuration folder")
set(CACHED_CONF_FILE ${CONF_FILE} CACHE STRING "If desired, you can build the application using\
the configuration settings specified in an alternate .conf file using this parameter. \
These settings will override the settings in the application’s .config file or its default .conf file.\
Multiple files may be listed, e.g. CONF_FILE=\"prj1.conf;prj2.conf\" \
The CACHED_CONF_FILE is internal Zephyr variable used between CMake runs. \
To change CONF_FILE, use the CONF_FILE variable.")
unset(CONF_FILE CACHE)

zephyr_file(CONF_FILES ${APPLICATION_CONFIG_DIR}/boards DTS APP_BOARD_DTS)

# The CONF_FILE variable is now set to its final value.
zephyr_boilerplate_watch(CONF_FILE)

zephyr_get(DTC_OVERLAY_FILE SYSBUILD LOCAL)
if(DTC_OVERLAY_FILE)
  # DTC_OVERLAY_FILE has either been specified on the cmake CLI or is already
  # in the CMakeCache.txt.
elseif(APP_BOARD_DTS)
  set(DTC_OVERLAY_FILE ${APP_BOARD_DTS})
elseif(EXISTS          ${APPLICATION_CONFIG_DIR}/${BOARD}.overlay)
  set(DTC_OVERLAY_FILE ${APPLICATION_CONFIG_DIR}/${BOARD}.overlay)
elseif(EXISTS          ${APPLICATION_CONFIG_DIR}/app.overlay)
  set(DTC_OVERLAY_FILE ${APPLICATION_CONFIG_DIR}/app.overlay)
endif()

set(DTC_OVERLAY_FILE ${DTC_OVERLAY_FILE} CACHE STRING "If desired, you can \
build the application using the DT configuration settings specified in an \
alternate .overlay file using this parameter. These settings will override the \
settings in the board's .dts file. Multiple files may be listed, e.g. \
DTC_OVERLAY_FILE=\"dts1.overlay dts2.overlay\"")

if(DEFINED SNIPPETS)
  # Search for (required) ${SNIPPET}.overlay and (optional) ${SNIPPET}.cmake files
  foreach(SNIPPET IN LISTS SNIPPETS_RAW_LIST)
    # Application level board snippets have precedence
    if(EXISTS     ${APPLICATION_SOURCE_DIR}/${BOARD}/snippets/${SNIPPET}.overlay)
      string(APPEND DTC_OVERLAY_FILE " ${APPLICATION_SOURCE_DIR}/boards/${BOARD}/snippets/${SNIPPET}.overlay")
      include(${APPLICATION_SOURCE_DIR}/${BOARD}/snippets/${SNIPPET}.cmake OPTIONAL NO_POLICY_SCOPE)
    elseif(EXISTS ${BOARD_DIR}/snippets/${SNIPPET}.overlay)
      string(APPEND DTC_OVERLAY_FILE " ${BOARD_DIR}/snippets/${SNIPPET}.overlay")
      include(${BOARD_DIR}/snippets/${SNIPPET}.cmake OPTIONAL NO_POLICY_SCOPE)
    else()
      message(FATAL_ERROR "Snippet ${SNIPPET} does not exist in ${APPLICATION_SOURCE_DIR}/${BOARD}/snippets or ${BOARD_DIR}/snippets")
    endif()
  endforeach()
endif()

# The DTC_OVERLAY_FILE variable is now set to its final value.
zephyr_boilerplate_watch(DTC_OVERLAY_FILE)
