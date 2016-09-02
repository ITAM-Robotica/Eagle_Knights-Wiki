cmake_minimum_required(VERSION 2.8.3)
project(tarea01)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs rospy)


catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(hello hello.cpp)
add_executable(pubvel pubvel.cpp)
add_executable(subpose subpose.cpp)
add_executable(tarea01 tarea01.cpp)

# Specify libraries against which to link.
target_link_libraries(hello ${catkin_LIBRARIES})
target_link_libraries(pubvel ${catkin_LIBRARIES})
target_link_libraries(subpose ${catkin_LIBRARIES})
target_link_libraries(tarea01 ${catkin_LIBRARIES})