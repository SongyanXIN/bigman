cmake_minimum_required(VERSION 2.8.3)
project(custom_effort_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  std_msgs
  message_generation
  control_msgs
)

 add_message_files(DIRECTORY msg
   FILES
   Command.msg
   CommandStamped.msg
   
   CommandArray.msg
   CommandArrayStamped.msg
   
   JointCommand.msg

)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})


generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   control_msgs
 )


# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    control_toolbox
    realtime_tools
    urdf
    forward_command_controller
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  )

add_library(${PROJECT_NAME}
  src/joint_effort_controller.cpp
  src/joint_velocity_controller.cpp
  src/joint_position_controller.cpp
  src/joint_group_effort_controller.cpp
  src/joint_position_effort_controller.cpp
  src/joint_group_position_effort_controller.cpp
)

add_dependencies(custom_effort_controllers custom_effort_controllers_generate_messages_cpp 
${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES effort_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
