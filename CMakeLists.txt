# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(simulator_art)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS angles geometry_msgs nav_msgs pcl_msgs pcl_ros pcl_conversions  roscpp roslib sensor_msgs std_msgs tf ackermann_msgs)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(FLTK REQUIRED)
if(FLTK_FOUND)
  # FindFLTK.cmake doesn't populate FLTK_INCLUDE_DIRS on some systems,
  # so we try to use FLTK_INCLUDE_DIR if we need to.
  if("${FLTK_INCLUDE_DIRS}" STREQUAL "")
    set(FLTK_INCLUDE_DIRS "${FLTK_INCLUDE_DIR}")
  endif()
  message(status "FLTK_LIBRARIES=" ${FLTK_LIBRARIES})
  message(status "FLTK_INCLUDE_DIR=" ${FLTK_INCLUDE_DIR})
  message(status "FLTK_INCLUDE_DIRS=" ${FLTK_INCLUDE_DIRS})
else()
  message(fatal_error "FLTK not found...aborting!")
endif()
set(Stage_DIR cmake)
find_package(Stage REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Stage_INCLUDE_DIRS}
  ${FLTK_INCLUDE_DIRS}
)

catkin_package(
    DEPENDS angles geometry_msgs nav_msgs  pcl_ros roscpp roslib sensor_msgs std_msgs tf ackermann_msgs
    CATKIN_DEPENDS # TODO
	INCLUDE_DIRS ${Stage_INCLUDE_DIRS}
	LIBRARIES ${Stage_LIBRARIES}
)

# new, improved simulation
add_executable(artstage src/artstage.cc src/vehicle_model.cc)
target_link_libraries(artstage   
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${FLTK_LIBRARIES}
  ${Stage_LIBRARIES}  -ldl)
  
if(catkin_EXPORTED_TARGETS)
  add_dependencies(artstage ${catkin_EXPORTED_TARGETS})
endif()

#simulator node that finds obstacles points from the SICK laser
add_executable(obstacles src/obstacles.cc)
target_link_libraries(obstacles   ${catkin_LIBRARIES} ${PCL_LIBRARIES})
