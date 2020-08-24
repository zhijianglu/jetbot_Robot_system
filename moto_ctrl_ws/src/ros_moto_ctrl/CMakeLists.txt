SET(PROJECT_NAME moto_ctrl)
cmake_minimum_required(VERSION 2.8.3)
project(${PROJECT_NAME})
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(OpenCV 3.4.6  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

set(LIBS
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${catkin_LIBRARIES}
        )

catkin_package(
  LIBRARIES imagePublisher
  CATKIN_DEPENDS cv_bridge image_transport  roscpp sensor_msgs std_msgs tf
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#todo control moto using keyboard
#add_executable(key_moto_ctrl src/main_key_moto_ctrl.cpp
#        src/KeyMotoCtrl.cpp
#        src/connection.cpp)
#
#target_link_libraries(key_moto_ctrl
#        ${LIBS}
#        )


#todo control moto using auto moving mode
#add_executable(auto_avoid src/main_auto_avoid.cpp
#        src/KeyMotoCtrl.cpp
#        src/SysCtrl.cpp
#        )
#
#target_link_libraries(auto_avoid
#        ${LIBS}
#        #        ${catkin_LIBRARIES}
#        )


add_executable(auto_avoid src/main_auto_avoid.cpp
        src/SysCtrl.cpp
        src/utils.cpp
        src/Searcher.cpp
)

target_link_libraries(auto_avoid
        ${LIBS}
        #        ${catkin_LIBRARIES}
        )


add_executable(codetest src/codetest.cpp
        )

target_link_libraries(codetest
        ${LIBS}
        #        ${catkin_LIBRARIES}
        )


add_executable(getData src/main_record_traing.cpp
        )

target_link_libraries(getData
        ${LIBS}
        #        ${catkin_LIBRARIES}
        )



add_executable(stereo src/main_stereo.cpp
        )

target_link_libraries(stereo
        ${LIBS}
        #        ${catkin_LIBRARIES}
        )

add_executable(bottle_searcher
        src/main_bottle_searcher.cpp
        src/Searcher.cpp

        )

target_link_libraries(bottle_searcher
        ${LIBS}
        #        ${catkin_LIBRARIES}
        )
