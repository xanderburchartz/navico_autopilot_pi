# ~~~
# Summary:      Local, non-generic plugin setup
# Copyright (c) 2020-2021 Mike Rossiter (template)
# Copyright (c) 2026 navico_autopilot_pi authors
# License:      GPLv3+
# ~~~

# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.


# -------- Options ----------

set(OCPN_TEST_REPO
    "opencpn/navico_autopilot-alpha"
    CACHE STRING "Default repository for untagged builds"
)
set(OCPN_BETA_REPO
    "opencpn/navico_autopilot-beta"
    CACHE STRING
    "Default repository for tagged builds matching 'beta'"
)
set(OCPN_RELEASE_REPO
    "opencpn/navico_autopilot-prod"
    CACHE STRING
    "Default repository for tagged builds not matching 'beta'"
)

#
#
# -------  Plugin setup --------
#
set(PKG_NAME navico_autopilot_pi)
set(PKG_VERSION 0.1.1) # Major.Minor.Patch
set(PKG_PRERELEASE "alpha")  # Empty, or a tag like 'beta'

set(DISPLAY_NAME "Navico Autopilot")    # Dialogs, installer artifacts, ...
set(PLUGIN_API_NAME NavicoAutopilot) # As of GetCommonName() in plugin API
set(PKG_SUMMARY "Control a B&G or Simrad (Navico) autopilot over NMEA 2000")
set(PKG_DESCRIPTION [=[
Controls a Navico autopilot computer (B&G and Simrad NAC-1/2/3, AC12/42)
directly over NMEA 2000: Standby, Auto (heading hold), Wind, Nav, No Drift
and course changes of 1 and 10 degrees. Shows the pilot mode, the heading
and the target heading or wind angle. Uses the NMEA 2000 connections of
OpenCPN itself, so any gateway OpenCPN can transmit through works. Each
button press sends one command.
]=])

set(PKG_AUTHOR "xanderburchartz")
set(PKG_IS_OPEN_SOURCE "yes")
set(PKG_HOMEPAGE https://github.com/xanderburchartz/navico_autopilot_pi)
set(PKG_INFO_URL https://github.com/xanderburchartz/navico_autopilot_pi)
# Tarballs of tagged builds are GitHub release assets; Metadata.cmake uses
# this for <tarball-url> instead of Cloudsmith.
set(PKG_GITHUB_RELEASES
  https://github.com/xanderburchartz/navico_autopilot_pi/releases/download)

set(SRC
    src/navico_autopilot_pi.h
    src/navico_autopilot_pi.cpp
    src/ap_panel.h
    src/ap_panel.cpp
    src/ap_prefs.h
    src/ap_prefs.cpp
    src/simnet.h
    src/simnet.cpp
    src/pilot_state.h
    src/pilot_state.cpp
)

set(PKG_API_LIB api-18)  #  A dir in opencpn-libs/ e. g., api-17 or api-16

# Minimum version which should be loaded. Ignored if api < 22.
# Allows plugins using functions from 1.22+ API to be loaded by
# hosts only supporting 1.21; plugins then needs to check if host
# supports 1.22+ before using such functions.
## set(MIN_API_VERSION 1.21)

macro(late_init)
  # Perform initialization after the PACKAGE_NAME library, compilers
  # and ocpn::api is available.
  target_include_directories(${PACKAGE_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)

  # Unit tests of the bus logic (no wxWidgets needed):
  # make test_simnet && ctest
  if (NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_executable(test_simnet EXCLUDE_FROM_ALL
      test/test_simnet.cpp src/simnet.cpp src/pilot_state.cpp)
    target_include_directories(test_simnet PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_compile_features(test_simnet PRIVATE cxx_std_17)
    add_test(NAME simnet COMMAND test_simnet)
  endif ()
endmacro ()

macro(add_plugin_libraries)
  # No extra libraries: only the plugin API and wxWidgets.
endmacro ()

