cmake_minimum_required(VERSION 3.0.2)
project(car_simulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)


###################################
## catkin specific configuration ##
###################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES car_simulator
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(car_sim src/car_sim.cpp src/car_sim_node.cpp src/car_ode.cpp)
target_link_libraries(car_sim ${catkin_LIBRARIES})

add_executable(simple_input src/simple_input.cpp src/simple_input_node.cpp)
target_link_libraries(simple_input ${catkin_LIBRARIES})
