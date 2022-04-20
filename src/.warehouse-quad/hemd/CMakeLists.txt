cmake_minimum_required(VERSION 2.8.3)
project(hemd)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  cmake_modules
  px_comm
  cv_bridge
  message_generation
)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
#find_package(PCL REQUIRED COMPONENTS common io)


add_message_files(
   FILES
   markerInfo.msg
   line.msg  
   quadstate.msg
 )
 
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   DEPENDS OpenCV Boost Eigen
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp message_runtime px_comm
)


include_directories(include
					${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(main src/main.cpp src/ekf.cpp src/sonar.cpp )
target_link_libraries(main zbar
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES})

add_dependencies(main ${catkin_EXPORTED_TARGETS})
add_executable(qr src/qr.cpp src/marker.cpp )
target_link_libraries(qr zbar
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES})

add_dependencies(qr ${catkin_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_EXPORTED_TARGETS})

