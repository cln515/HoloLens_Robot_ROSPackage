project(pepper2holo_calib)

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

add_executable(pepper2holo_calib
    main.cpp
)

target_link_libraries(pepper2holo_calib
  ${catkin_LIBRARIES}
  boost_system
  crypto
  ssl
  cpprest
  uuid
)

add_dependencies(pepper2holo_calib ${fujio_2dnav_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})