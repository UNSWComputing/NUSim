cmake_minimum_required(VERSION 3.22)
project(
  k1_mujoco_sim
  VERSION 0.1.0
  LANGUAGES C CXX
)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Self-contained MuJoCo replacement for the Webots+mck K1 simulation. Prerequisite: tools/install_deps.sh (installs
# MuJoCo + Fast-DDS into .deps/install).

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "" FORCE
  )
endif()

# Dependency prefix: the docker image (docker/Dockerfile) bakes deps into /opt/k1sim-deps; a native fallback build
# (tools/install_deps.sh) uses .deps/install.
if(DEFINED ENV{K1SIM_DEPS_PREFIX})
  list(APPEND CMAKE_PREFIX_PATH "$ENV{K1SIM_DEPS_PREFIX}")
endif()
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.deps/install" "/opt/k1sim-deps")

include(cmake/FetchNUClear.cmake)
include(cmake/FetchJson.cmake)
include(cmake/FetchGlfw.cmake)
include(cmake/MuJoCoTarget.cmake)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED) # Fast-DDS 2.x exports the 'fastrtps' package

# Single include root: every include is written relative to mujoco/ (e.g. "shared/k1/JointIndex.hpp",
# "module/Simulation/src/Simulation.hpp").
add_library(k1sim_shared INTERFACE)
target_include_directories(k1sim_shared INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(k1sim_shared INTERFACE mujoco::mujoco NUClear::nuclear yaml-cpp::yaml-cpp Threads::Threads)
# XInitThreads() (shared/gl/XThreads.hpp) is called by every role's main to make Xlib thread-safe for the Viewer +
# Camera GL threads; link X11 for the symbol.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  find_package(X11)
  if(X11_FOUND)
    target_link_libraries(k1sim_shared INTERFACE ${X11_LIBRARIES})
  endif()
endif()
target_compile_definitions(k1sim_shared INTERFACE K1SIM_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(module/ConsoleLog)
add_subdirectory(module/Simulation)
add_subdirectory(module/SdkBridge)
add_subdirectory(module/Locomotion)
add_subdirectory(module/Camera)
add_subdirectory(module/Supervisor)
add_subdirectory(module/Viewer)

# --- Roles → bin/<role_path>/<role_name> (see cmake/K1SimRole.cmake) ---
set(K1SIM_ROLE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/K1SimRole.cmake)
file(GLOB_RECURSE role_files CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/roles/*.role")
foreach(role_file ${role_files})
  file(RELATIVE_PATH rel "${CMAKE_CURRENT_SOURCE_DIR}/roles" "${role_file}")
  get_filename_component(role_name "${rel}" NAME_WE)
  get_filename_component(role_path "${rel}" DIRECTORY)
  if(role_path)
    string(REPLACE "/" "-" role_prefix "${role_path}")
    set(role "${role_prefix}-${role_name}")
  else()
    set(role "${role_name}")
  endif()
  option("ROLE_${role}" "Build the ${role} role" ON)
  if(ROLE_${role})
    include("${role_file}")
  endif()
endforeach()

# Legacy single-binary entry (kept during the roles transition; exercised by the contract test). Equivalent to the
# sim/soccer role minus Camera.
add_executable(k1_mujoco_sim src/main.cpp)
target_link_libraries(
  k1_mujoco_sim
  PRIVATE k1sim_module_consolelog
          k1sim_module_simulation
          k1sim_module_sdkbridge
          k1sim_module_locomotion
          k1sim_module_supervisor
          k1sim_module_viewer
          k1sim_shared
)

enable_testing()
add_subdirectory(test/unit)
