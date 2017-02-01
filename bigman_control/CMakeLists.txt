cmake_minimum_required(VERSION 2.8.3)
project(bigman_control)

set(CMAKE_CXX_FLAGS "-std=c++11 -w")


find_package(catkin REQUIRED COMPONENTS
        custom_effort_controllers
        roscpp
        rospy
        std_msgs
        sensor_msgs
        urdf
        tf
        tf_conversions
        eigen_conversions
        ecl_geometry
        )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(RBDL COMPONENTS URDFReader REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs)


include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})


add_library(bigman_kinematic
        src/bigman_kinematic/bigman_kinematic.cpp)

add_library(bigman_driver
        src/bigman_driver/bigman_driver.cpp)

add_library(bigman_low_level_controller
        src/bigman_low_level_controller/bigman_low_level_controller.cpp)

add_library(inverse_dynamic_controller
        src/inverse_dynamic_controller/inverse_dynamic_controller.cpp)



add_executable(test_bigman_driver test/test_bigman_driver.cpp)
target_link_libraries(test_bigman_driver
        bigman_driver
        ${catkin_LIBRARIES}
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY})

add_executable(test_bigman_low_level_controller test/test_bigman_low_level_controller.cpp)
target_link_libraries(test_bigman_low_level_controller
        bigman_low_level_controller
        bigman_driver
        bigman_kinematic
        ${catkin_LIBRARIES}
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY})

add_executable(test_inverse_dynamic_controller test/test_inverse_dynamic_controller.cpp)
target_link_libraries(test_inverse_dynamic_controller
        inverse_dynamic_controller
        bigman_driver
        bigman_kinematic
        ${catkin_LIBRARIES}
        ${RBDL_LIBRARY}
        ${RBDL_URDFReader_LIBRARY})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
