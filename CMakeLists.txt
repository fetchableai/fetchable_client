cmake_minimum_required(VERSION 2.8.3)
project(fetchable_client)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
)

add_service_files(
  FILES
  Fetch.srv
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fetchable_client
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example_caller_cpp tests/example_caller_cpp.cpp)

target_link_libraries(example_caller_cpp
  ${catkin_LIBRARIES}
)
