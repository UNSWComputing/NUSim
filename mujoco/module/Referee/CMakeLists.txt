add_library(k1sim_module_referee STATIC src/Referee.cpp)
target_link_libraries(k1sim_module_referee PUBLIC k1sim_shared ${CMAKE_DL_LIBS})
