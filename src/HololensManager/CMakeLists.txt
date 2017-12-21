project(hololensManager)

add_definitions(-std=c++11)

find_package(OpenSSL REQUIRED)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  .
  /usr/local/includes/
  ${catkin_INCLUDE_DIRS}
)

message("----------------------"  ${INCLUDE_DIRS})
message("----------------------"  ${catkin_INCLUDE_DIRS})
message("----------------------"  ${PROJECT_NAME})

add_executable(hololensManager
    main.cpp
    linearSolver.cpp
    HololensManager.cpp
)

target_link_libraries(hololensManager
  ${catkin_LIBRARIES}
  boost_system
  crypto
  ssl
  cpprest
  uuid
  opencv_core
  opencv_highgui
  opencv_imgproc  
)

add_dependencies(hololensManager ${fujio_2dnav_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})