# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(agitr)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs turtlesim std_srvs)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(move_fwd move_fwd.cpp)

# Specify libraries against which to link.
target_link_libraries(move_fwd ${catkin_LIBRARIES})

