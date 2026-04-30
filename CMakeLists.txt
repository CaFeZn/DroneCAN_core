target_sources(xr PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src/DroneCANNode.cpp
  ${CMAKE_CURRENT_LIST_DIR}/src/dronecan_frame.cpp
  ${CMAKE_CURRENT_LIST_DIR}/third_party/canard.c
)

target_include_directories(xr PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${CMAKE_CURRENT_LIST_DIR}/third_party
)
