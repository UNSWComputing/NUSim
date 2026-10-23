# --- k1sim_idl: Fast-DDS type support generated from mujoco/idl/ at build time ---
# Clean-room IDL for the Booster SDK wire types; see PROTOCOL.md for provenance. The generated code lives in the build
# directory and is never committed — cmake/IdlGen.cmake runs idl/regenerate.sh and exports k1sim_idl_outputs /
# k1sim_idl_packages. Adding a message means adding one .idl; nothing in this file changes.
include(${PROJECT_SOURCE_DIR}/cmake/IdlGen.cmake)

# Each <Type>CdrAux.ipp holds explicit fastcdr template specializations
# (calculate_serialized_size/serialize/deserialize/serialize_key for that exact type) that <Type>PubSubTypes.cxx calls
# but neither declares nor includes the definition of — fastddsgen expects them compiled as their own translation unit.
# CMake doesn't recognise ".ipp" as a C++ source extension by default, so mark it explicitly (each file's include guard
# + one entry per type in k1sim_idl_outputs keeps this to exactly one definition per type across the whole link).
foreach(generated ${k1sim_idl_outputs})
  if(generated MATCHES "\\.ipp$")
    set_source_files_properties(${generated} PROPERTIES LANGUAGE CXX)
  endif()
endforeach()

add_library(k1sim_idl STATIC ${k1sim_idl_outputs})
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  # On case-insensitive volumes (including Docker bind mounts), generated Time.h shadows system <time.h>. Consumer
  # includes use quotes; keep these dirs out of angle-bracket searches.
  foreach(pkg ${k1sim_idl_packages})
    target_compile_options(k1sim_idl PUBLIC "-iquote${pkg}")
  endforeach()
else()
  target_include_directories(k1sim_idl PUBLIC ${k1sim_idl_packages})
endif()
target_link_libraries(k1sim_idl PUBLIC fastrtps fastcdr)

# --- module::SdkBridge ---
add_library(
  k1sim_module_sdkbridge STATIC
    src/BallCommandReader.cpp
    src/DdsParticipant.cpp
    src/GroundTruthPublisher.cpp
    src/OdometryModel.cpp
    src/RpcDispatch.cpp
    src/RpcServer.cpp
    src/SdkBridge.cpp
    src/StatePublisher.cpp
)
target_link_libraries(
  k1sim_module_sdkbridge
  PUBLIC k1sim_shared k1sim_idl
  PRIVATE fastrtps fastcdr nlohmann_json::nlohmann_json
)

add_subdirectory(test_support)
