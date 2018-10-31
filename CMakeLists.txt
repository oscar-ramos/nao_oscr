CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(nao_oscr)

SET(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(OSCR oscr REQUIRED)

FIND_PACKAGE(catkin REQUIRED COMPONENTS
  sensor_msgs
  naoqi_bridge_msgs
  roscpp
  rospack
  roslib
  rospy
)

catkin_python_setup()
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

INCLUDE_DIRECTORIES(
  include
  ${OSCR_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  )

LINK_DIRECTORIES(
  ${OSCR_LIBRARY_DIRS}
  )

# ADD_LIBRARY(nao_oscr SHARED
#   src/utils/marker.cpp
#   src/utils/ball-marker.cpp
#   src/utils/frame-marker.cpp
#   src/utils/marker-utils.cpp
#   src/utils/joint-state-pub.cpp
#   src/utils/robot-interface.cpp
#   )

# # Mark cpp header files for installation
# INSTALL(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
# )

# SET(ALL_LIBRARIES
#   nao_oscr
#   ${catkin_LIBRARIES}
#   ${OSCR_LIBRARIES}
# )

# ADD_EXECUTABLE(naoModelTest src/nao-model-test.cpp)
# TARGET_LINK_LIBRARIES(naoModelTest ${ALL_LIBRARIES})

# ADD_EXECUTABLE(naoMotionRviz src/nao-motion-rviz.cpp)
# TARGET_LINK_LIBRARIES(naoMotionRviz ${ALL_LIBRARIES})

# ADD_EXECUTABLE(naoMotionNaoqi src/nao-motion-naoqi.cpp)
# TARGET_LINK_LIBRARIES(naoMotionNaoqi ${ALL_LIBRARIES})
