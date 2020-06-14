cmake_minimum_required(VERSION 2.8.3)
project(chapter5_tutorials)

find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      message_generation
      sensor_msgs
      dynamic_reconfigure
      driver_base
      cv_bridge
      image_transport
      camera_info_manager
      tf)

find_package(OpenCV 2 REQUIRED)

generate_dynamic_reconfigure_options(
    cfg/Threshold.cfg
    cfg/Camera.cfg
    cfg/CameraStereo.cfg
    cfg/Homography.cfg)

catkin_package(
    CATKIN_DEPENDS
      roscpp
      message_runtime
      sensor_msgs
      dynamic_reconfigure
      driver_base
      cv_bridge
      image_transport
      camera_info_manager
      tf)

include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(camera3 src/camera3.cpp)
add_dependencies(camera3 ${PROJECT_NAME}_gencfg)
target_link_libraries(camera3 ${catkin_LIBRARIES})

add_executable(camera3_1 src/camera3_1.cpp)
add_dependencies(camera3_1 ${PROJECT_NAME}_gencfg)
target_link_libraries(camera3_1 ${catkin_LIBRARIES})


add_executable(camera2 src/camera2.cpp)
add_dependencies(camera2 ${PROJECT_NAME}_gencfg)
target_link_libraries(camera2 ${catkin_LIBRARIES})

add_executable(camera src/camera.cpp)
add_dependencies(camera ${PROJECT_NAME}_gencfg)
target_link_libraries(camera ${catkin_LIBRARIES})

add_executable(camera_timer src/camera_timer.cpp)
target_link_libraries(camera_timer ${catkin_LIBRARIES})

add_executable(camera_polling src/camera_polling.cpp)
target_link_libraries(camera_polling ${catkin_LIBRARIES})

add_executable(camera_stereo src/camera_stereo.cpp)
add_dependencies(camera_stereo ${PROJECT_NAME}_gencfg)
target_link_libraries(camera_stereo ${catkin_LIBRARIES})

#add_executable(homography src/homography.cpp)
#add_dependencies(homography ${PROJECT_NAME}_gencfg)
#target_link_libraries(homography ${catkin_LIBRARIES} ${OpenCV2_LIBRARIES})

install(TARGETS camera camera2 camera3
camera3_1  camera_timer camera_polling camera_stereo 
#homography
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir bag config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
