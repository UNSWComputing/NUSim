# Every test_*.cpp here becomes a ctest target automatically — workstreams add test files without touching this
# CMakeLists (avoids concurrent-edit conflicts).
file(GLOB test_sources CONFIGURE_DEPENDS "test_*.cpp")

foreach(test_src ${test_sources})
  get_filename_component(test_name ${test_src} NAME_WE)
  add_executable(${test_name} ${test_src})
  target_link_libraries(
    ${test_name} PRIVATE k1sim_shared k1sim_module_consolelog k1sim_module_simulation k1sim_module_sdkbridge
                         k1sim_module_locomotion k1sim_module_viewer
  )
  add_test(NAME ${test_name} COMMAND ${test_name})
  set_tests_properties(${test_name} PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endforeach()
set_tests_properties(test_dds_transport PROPERTIES TIMEOUT 20)
set_tests_properties(test_multi_robot_dds PROPERTIES TIMEOUT 30)

# macOS offscreen rendering must work on a worker thread without a window server API.
if(APPLE)
  find_package(Boost REQUIRED)
  add_executable(test_camera_context ../platform/test_camera_context.cpp)
  target_link_libraries(test_camera_context PRIVATE k1sim_module_camera Boost::headers)
  add_test(NAME test_camera_context COMMAND test_camera_context)
  set_tests_properties(test_camera_context PROPERTIES SKIP_RETURN_CODE 77)
endif()
