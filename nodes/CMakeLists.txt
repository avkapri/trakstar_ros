add_executable(trakstar_node trakstar_node.cpp)
add_dependencies(trakstar_node trakstar)

target_link_libraries(trakstar_node
	trakstar
	${catkin_LIBRARIES}
)


add_executable(trakstar_calib
  trakstar_calibrate.cpp)
add_dependencies(trakstar_calib
  trakstar)

target_link_libraries(trakstar_calib
  trakstar
  ${catkin_LIBRARIES}
)


add_executable(trakstar_calib_coords
  trakstar_calibrate_coords.cpp)
add_dependencies(trakstar_calib_coords
  trakstar)

target_link_libraries(trakstar_calib_coords
  trakstar
  ${catkin_LIBRARIES}
)

install(TARGETS trakstar_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS trakstar_calib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS trakstar_calib_coords
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

