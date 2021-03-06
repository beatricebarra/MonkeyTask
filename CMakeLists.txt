cmake_minimum_required(VERSION 2.8.3)
project(monkeytask)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -I /usr/include/SDL2 -lSDL2)
add_compile_options(-I /usr/include/SDL2 -lSDL2)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mathlib
  robotlib
    skinematics
     motion_generators
     kuka_fri_bridge
     sg_differentiation    
)

## System dependencies are found with CMake's conventions
 find_package(SDL REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ThreeDReaching/include monkeytask_test_1/include monkeytask_test_2/include SlowMovementApplication/include monkeytaskSCI/include monkeytask_arb/include monkeytask_arb_visual/include monkeytask_arb_data/include
  LIBRARIES monkeytask SDL2 
  CATKIN_DEPENDS mathlib robotlib
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(monkeytask_test_1/include) 
include_directories(monkeytask_test_2/include)
include_directories(SlowMovementApplication/include)
include_directories(monkeytask_arb/include)
include_directories(monkeytask_arb_data/include)
include_directories(monkeytaskSCI/include)
include_directories(monkeytask_arb_visual/include)
include_directories(ThreeDReaching/include)

## Here include directories. BB 

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/monkeytask.cpp
# )


FILE(GLOB theSourceFiles1 monkeytask_test_1/src/*.cpp)
add_library(monkeytask_test_1 ${theSourceFiles1})

FILE(GLOB theSourceFiles2 monkeytask_test_2/src/*.cpp)
add_library(monkeytask_test_2 ${theSourceFiles2})

FILE(GLOB theSourceFiles3 SlowMovementApplication/src/*.cpp)
add_library(SlowMovementApplication ${theSourceFiles3})

FILE(GLOB theSourceFiles4 monkeytask_arb/src/*.cpp)
add_library(monkeytask_arb ${theSourceFiles4})

FILE(GLOB theSourceFiles5 monkeytaskSCI/src/*.cpp)
add_library(monkeytaskSCI ${theSourceFiles5})

FILE(GLOB theSourceFiles6 monkeytask_arb_visual/src/*.cpp)
add_library(monkeytask_arb_visual ${theSourceFiles6})

FILE(GLOB theSourceFiles7 monkeytask_arb_data/src/*.cpp)
add_library(monkeytask_arb_data ${theSourceFiles7})

FILE(GLOB theSourceFiles8 ThreeDReaching/src/*.cpp)
add_library(ThreeDReaching ${theSourceFiles8})


## Here add library. BB 


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/monkeytask_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

 target_link_libraries(monkeytask_test_1
   ${catkin_LIBRARIES} SDL2
 )

 target_link_libraries(monkeytask_test_2
   ${catkin_LIBRARIES} SDL2
 )

target_link_libraries(SlowMovementApplication
   ${catkin_LIBRARIES} SDL2
)
   
target_link_libraries(monkeytask_arb
   ${catkin_LIBRARIES} SDL2
)

target_link_libraries(monkeytaskSCI
   ${catkin_LIBRARIES} SDL2
)

target_link_libraries(monkeytask_arb_visual
   ${catkin_LIBRARIES} SDL2
)

target_link_libraries(monkeytask_arb_data
   ${catkin_LIBRARIES} SDL2
)

target_link_libraries(ThreeDReaching
   ${catkin_LIBRARIES} SDL2
)

   
 ## Here sound library link. BB   

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_monkeytask.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
###################################
## RobotToolKit specific stuff  ###
###################################
# to comply with the file structure of RobotToolKit, you need to create symlinks 
# to any compiled modules in the RobotToolKit/module folder. This is automated below.
# just add the modules of this package as set(rtk_modules myAwesomeModule1 myAwesomeModule2)
# modules can be world or robot modules. Note that you need to specify your targets using 
# standard cmake/catkin procedures above. This is step is only for linking the build modules.
#set(rtk_modules monkeytask_test_1 )
 set(rtk_modules ThreeDReaching monkeytask_test_1 monkeytask_test_2 SlowMovementApplication monkeytask_arb monkeytaskSCI monkeytask_arb_visual monkeytask_arb_data) ## Here add application name. BB 
execute_process(COMMAND rospack find rtk_pkg_tools OUTPUT_VARIABLE rtk_pkg_tools_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
include(${rtk_pkg_tools_DIR}/rtk_create_symlinks.cmake)

# Uncomment the following line if you want catkin dependencies you specify in CMakeLists.txt to be automatically copied to the package.xml
execute_process(COMMAND ${rtk_pkg_tools_DIR}/scripts/CopyDepsFromCMakeListsToPackageXML.py ${PROJECT_NAME})

