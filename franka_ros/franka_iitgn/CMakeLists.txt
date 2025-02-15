cmake_minimum_required(VERSION 3.4)
project(franka_iitgn)

# Set build type and C++ standard
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find catkin and required components (only add what you need)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  controller_interface
  franka_hw
  hardware_interface
  sensor_msgs
  std_msgs
)

# If you use additional packages in your code, add them here.
# For example, if you use geometry_msgs or tf:
# find_package(catkin REQUIRED COMPONENTS geometry_msgs tf)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES franka_iitgn
  CATKIN_DEPENDS roscpp pluginlib controller_interface franka_hw hardware_interface
)

# Specify include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Build the library (only your controller source file)
add_library(franka_iitgn
  src/cartesian_controller_back.cpp
  src/cartesian_controller_front.cpp
  src/cartesian_pose_controller.cpp
)

# Link the required libraries
target_link_libraries(franka_iitgn
  ${catkin_LIBRARIES}
)

# (Optional) Install the library and any header files
install(TARGETS franka_iitgn
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# (Optional) If you're using a plugin description file for pluginlib,
# install it so that the controller manager can find it.
install(FILES config/franka_iitgn_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Install Python scripts
catkin_install_python(PROGRAMS 
  src/joint_state_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  src/close_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)