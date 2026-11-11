find_package(OpenGL REQUIRED)

add_library(k1sim_module_viewer STATIC src/Viewer.cpp src/ViewCamera.cpp)
target_link_libraries(
  k1sim_module_viewer
  PUBLIC k1sim_shared
  PRIVATE glfw OpenGL::GL k1sim_imgui
)
