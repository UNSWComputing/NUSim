# Dear ImGui for the viewer's on-screen camera controls (buttons/combos drawn into the GLFW window on top of the MuJoCo
# render). ImGui ships no CMake build, so compile the core plus the GLFW + legacy OpenGL2 backends here. OpenGL2 matches
# MuJoCo's own fixed-function mjr context, so no GL loader is needed.
include(FetchContent)

FetchContent_Declare(imgui URL https://github.com/ocornut/imgui/archive/refs/tags/v1.91.8.tar.gz)
FetchContent_MakeAvailable(imgui)

add_library(
  k1sim_imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
)
# MuJoCo's own include directory ships an imgui.h of a different version (for its studio app). Plain -I (not SYSTEM)
# dirs are searched before MuJoCo's -isystem one, so users of this target get the headers matching this library.
# module/Viewer static_asserts the version.
target_include_directories(k1sim_imgui BEFORE PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_compile_definitions(k1sim_imgui PUBLIC K1SIM_IMGUI_VERSION_NUM=19180)
find_package(OpenGL REQUIRED)
target_link_libraries(k1sim_imgui PUBLIC glfw OpenGL::GL)
