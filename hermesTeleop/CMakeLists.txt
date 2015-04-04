cmake_minimum_required(VERSION 2.8.3)
project(hermesTeleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  joy
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(src)

#add_executable(HelloEposCmd src/HelloEposCmd.cpp)
#target_link_libraries(HelloEposCmd EposCmd ${catkin_LIBRARIES})

add_executable(hermesTeleopNode src/hermesTeleop.cpp)
target_link_libraries(hermesTeleopNode EposCmd ${catkin_LIBRARIES})
add_executable(hermesTwistTeleopNode src/hermesTwistTeleopNode.cpp)
target_link_libraries(hermesTwistTeleopNode EposCmd ${catkin_LIBRARIES})
add_executable(DoTheTwistNode src/DoTheTwistNode.cpp)
target_link_libraries(DoTheTwistNode ${catkin_LIBRARIES})
