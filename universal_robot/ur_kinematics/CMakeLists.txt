cmake_minimum_required(VERSION 2.8.3)
project(ur_kinematics)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
  boost_numpy 
  rospy
  roscpp 
  geometry_msgs 
  pluginlib 
  tf_conversions  
)

find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ur_kin _ur_kin_wrapper_cpp
  CATKIN_DEPENDS 
    roscpp geometry_msgs pluginlib tf_conversions 
    boost_numpy
  # DEPENDS system_lib
)

###########
## Build ##
###########

# include Boost and Python.
include_directories(
              include 
              ${catkin_INCLUDE_DIRS}
              ${Boost_INCLUDE_DIR}
              ${PYTHON_INCLUDE_DIRS}
              )
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

## Declare a cpp library
add_library(ur_kin src/ur_kin.cpp)
add_library(_ur_kin_wrapper_cpp src/ur_kin_wrapper.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(ur_kin ${catkin_LIBRARIES})
target_link_libraries(_ur_kin_wrapper_cpp ur_kin ${PYTHON_LIBRARIES} boost_numpy ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(_ur_kin_wrapper_cpp PROPERTIES 
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)


