# Copyright (c) 2026 Open Device Partnership and Contributors
# SPDX-License-Identifier: MIT

zephyr_library()
zephyr_library_sources(pdg_i2c.c)
zephyr_library_include_directories(${CMAKE_CURRENT_LIST_DIR})

# Resolve the prebuilt Pico de Gallo FFI artifacts on the host. By default this assumes
# the module lives at <repo>/zephyr and the cargo artifacts are under
# <repo>/target/release. Override with -DPICO_DE_GALLO_DIR=<repo> if needed.
if(DEFINED PICO_DE_GALLO_DIR)
  set(_pdg_root "${PICO_DE_GALLO_DIR}")
elseif(DEFINED ENV{PICO_DE_GALLO_DIR})
  set(_pdg_root "$ENV{PICO_DE_GALLO_DIR}")
else()
  set(_pdg_root "${ZEPHYR_PICO_DE_GALLO_MODULE_DIR}")
endif()

set(_pdg_libdir "${_pdg_root}/target/release")

find_library(PDG_FFI_LIB
  NAMES pico_de_gallo_ffi
  PATHS "${_pdg_libdir}"
  NO_DEFAULT_PATH
)
if(NOT PDG_FFI_LIB)
  message(FATAL_ERROR
    "Pico de Gallo FFI library not found in ${_pdg_libdir}.\n"
    "Build it first with: cargo build --release -p pico-de-gallo-ffi")
endif()

file(GLOB _pdg_hdr
  "${_pdg_root}/target/release/build/pico-de-gallo-ffi-*/out/include/pico_de_gallo.h")
if(NOT _pdg_hdr)
  message(FATAL_ERROR
    "Generated pico_de_gallo.h not found under ${_pdg_root}/target/release/build.\n"
    "Build the FFI crate first with: cargo build --release -p pico-de-gallo-ffi")
endif()
list(GET _pdg_hdr 0 _pdg_hdr0)
get_filename_component(_pdg_incdir "${_pdg_hdr0}" DIRECTORY)

# The bottom file runs in the native simulator (host) context, where it may
# include the host-only pico_de_gallo.h and link against the FFI shared object.
target_sources(native_simulator INTERFACE ${CMAKE_CURRENT_LIST_DIR}/pdg_i2c_bottom.c)
target_compile_options(native_simulator INTERFACE
  "-I${_pdg_incdir}"
  "-I${CMAKE_CURRENT_LIST_DIR}")
target_link_options(native_simulator INTERFACE
  "-L${_pdg_libdir}"
  "-lpico_de_gallo_ffi"
  "-Wl,-rpath,${_pdg_libdir}")
