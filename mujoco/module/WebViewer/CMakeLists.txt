add_library(k1sim_module_webviewer STATIC src/WebViewer.cpp src/WebStream.cpp)
target_link_libraries(k1sim_module_webviewer PUBLIC k1sim_shared)
