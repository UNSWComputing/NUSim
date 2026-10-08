add_library(k1sim_module_simulation STATIC src/SimCore.cpp src/Simulation.cpp)
target_link_libraries(
  k1sim_module_simulation
  PUBLIC k1sim_shared
  PRIVATE nlohmann_json::nlohmann_json
)
