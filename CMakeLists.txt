cmake_minimum_required(VERSION 2.8.3)
project(hololens_robot_navigation)

 add_definitions(-std=c++11)

 
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)
find_package(Ceres REQUIRED
)

catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(hololens_robot_navigation
    src/HololensManager/main.cpp
    src/HololensManager/HololensManager.cpp
    src/HololensManager/linearSolver.cpp
)

target_link_libraries(hololens_robot_navigation
  ${catkin_LIBRARIES}
  opencv_core
  opencv_highgui
  opencv_imgproc  
)

add_executable(robot2holo_calibration
	src/robot2holo_calib/main.cpp
	src/robot2holo_calib/teleop.cpp
	src/robot2holo_calib/calibrator.cpp
	src/robot2holo_calib/nonlinear_solver.cpp
)
target_link_libraries(robot2holo_calibration
  ${catkin_LIBRARIES}
  ceres
)