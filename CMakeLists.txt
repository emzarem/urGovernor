cmake_minimum_required(VERSION 2.8.3)
project(urGovernor)

## Use C++11
add_definitions(-std=c++11)
## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
#add_definitions(-std=c++11 -Wall -Werror)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    serial
    urVision
    message_generation
)

## Find system libraries
#find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED)\

############################################
## Message / Service / Action -- Definitions
###########################################
add_service_files(
  DIRECTORY
  srv
  FILES
  FetchWeed.srv
  MarkUprooted.srv
  SerialWrite.srv
  SerialRead.srv
  KinematicsTest.srv
)

generate_messages(
  DEPENDENCIES
  urVision
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
#    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    serial
    urVision
#  DEPENDS
#    Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/kinematics
  include/shared
  ${catkin_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIR}
#  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
# Add kinematics 
add_library(${PROJECT_NAME}_core
  src/kinematics/deltaRobot.cpp
)

## Declare cpp executables
add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}_node.cpp
)

## Add dependencies to exported targets, like ROS msgs or srvs
add_dependencies(${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

## For deltaTest_node

## Declare cpp executables
add_executable(deltaTest
  src/testnodes/deltaTest_node.cpp
)

add_dependencies(deltaTest
  ${catkin_EXPORTED_TARGETS}
)

# Link libraries
target_link_libraries(deltaTest
  ${catkin_LIBRARIES}
)

## Node for processing serial output
add_executable(serialOutput src/serialOutput_node.cpp)

add_dependencies(serialOutput
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(serialOutput
  ${catkin_LIBRARIES}
)

## For testing serial
add_executable(testTeensy
    src/testnodes/testTeensy_node.cpp
)

add_dependencies(testTeensy
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(testTeensy
  ${catkin_LIBRARIES}
)

## For kinematics manual testing

## Declare cpp executables
add_executable(kinematicsTest
  src/testnodes/kinematics_test_node.cpp
)

add_dependencies(kinematicsTest
  ${PROJECT_NAME}_core
  ${catkin_EXPORTED_TARGETS}
)

# Link libraries
target_link_libraries(kinematicsTest
  ${PROJECT_NAME}_core
  ${catkin_LIBRARIES}
)

## Stub node for testing without physical serial lines
add_executable(serialStub
    src/testnodes/serialStub_node.cpp
)

add_dependencies(serialStub
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(serialStub
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_urGovernor.cpp
)
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_core)
endif()
