cmake_minimum_required(VERSION 2.8)
       include(CheckCXXCompilerFlag)
      CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
      CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
      if(COMPILER_SUPPORTS_CXX11)
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
      elseif(COMPILER_SUPPORTS_CXX0X)
          set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
      else()
              message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
      endif()

      if(COMMAND cmake_policy)
        cmake_policy(SET CMP0003 NEW)
      endif(COMMAND cmake_policy)

project(ar_localization)
#set(OpenCV_DIR /home/agv/OpenCV/opencv-2.4.9)
########################################################################

########################################################################

find_package( OpenCV REQUIRED )
message("OPENCV_INC=${OpenCV_INCLUDE_DIRS}")
message("OPENCV_INC=${OpenCV_LIBS}")

include_directories(${catkin_INCLUDE_DIRS})
find_package(catkin REQUIRED COMPONENTS  roscpp rospy std_msgs message_generation nav_msgs cv_bridge  image_transport   sensor_msgs   std_msgs)
add_message_files(  FILES  arMessage.msg  MarkGpos.msg)
generate_messages(  DEPENDENCIES  std_msgs)
catkin_package()

set(AR_SOURCE_DIR ${PROJECT_SOURCE_DIR})
include_directories(${AR_SOURCE_DIR}/include/ARToolKitPlus ${AR_SOURCE_DIR}/include ${AR_SOURCE_DIR}/include/ARToolKitPlus/extra)
include_directories(${AR_SOURCE_DIR}/qrslam)

include_directories(${catkin_INCLUDE_DIRS} )
link_directories(${AR_SOURCE_DIR}/lib ${catkin_LIBRARIES} )

# ARToolkitPlus core files (headers and sources)
file(GLOB_RECURSE HEADERS_AR
        ${AR_SOURCE_DIR}/include/ARToolKitPlus/*.h        ${AR_SOURCE_DIR}/camera_calibration/class/*.h
        ${AR_SOURCE_DIR}/qrslam/class/*.h                 ${AR_SOURCE_DIR}/qrslam/*.h
        ${AR_SOURCE_DIR}/image_convert/*.h
)

file(GLOB SOURCES_AR
        ${AR_SOURCE_DIR}/src/*.cpp              ${AR_SOURCE_DIR}/src/core/*.cpp
        ${AR_SOURCE_DIR}/src/extra/*.cpp        ${AR_SOURCE_DIR}/src/librpp/*.cpp
        ${AR_SOURCE_DIR}/camera_calibration/class/*.cpp        ${AR_SOURCE_DIR}/qrslam/class/*.cpp
        ${AR_SOURCE_DIR}/qrslam/qrslam.cpp        ${AR_SOURCE_DIR}/qrslam/odompub/*.cpp
        ${AR_SOURCE_DIR}/image_convert/*.cpp
        )

#  add_library(ARToolKitPlus SHARED ${HEADERS_AR} ${SOURCES_AR} ${catkin_LIBRARIES} )
#  target_link_libraries( ARToolKitPlus ${OpenCV_LIBS} ${catkin_LIBRARIES} )
#
#  add_executable(camera_calibration ${AR_SOURCE_DIR}/camera_calibration/calibrate.cpp )
#  target_link_libraries(camera_calibration ${OpenCV_LIBS} ARToolKitPlus ${catkin_LIBRARIES}  )
#
#  add_executable(ar_slam_main ${AR_SOURCE_DIR}/qrslam/main.cpp )
#  target_link_libraries(ar_slam_main ${OpenCV_LIBS} ARToolKitPlus ${catkin_LIBRARIES} )
##########
#add_executable(odompub ${AR_SOURCE_DIR}/qrslam/pub/odompub.cpp )
#target_link_libraries(odompub    ${catkin_LIBRARIES}  )
#
#add_executable(odom_listen ${AR_SOURCE_DIR}/qrslam/pub/odom_listen.cpp )
#target_link_libraries(odom_listen    ${catkin_LIBRARIES}  )

#ADD_SUBDIRECTORY(tutorial_slam2d)

add_executable(vel_pub ${AR_SOURCE_DIR}/qrslam/pub/velpub.cpp )
target_link_libraries(vel_pub    ${catkin_LIBRARIES}  )



###########################g2o##################################################################

###########################g2o##################################################################
message("CMAKE_MODULE_PATH " ${CMAKE_MODULE_PATH} )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${AR_SOURCE_DIR}/tutorial_slam2d/cmake-modules/")
message("CMAKE_MODULE_PATH "  CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

######g2o #############
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
message("G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR}")
link_directories(${G2O_LIBRARY_DIRS})
link_libraries(${G2O_LIBRARIES})
message("G2o-libraries ${G2O_LIBRARIES}")
#######CSparse ##########
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
message("CSPARSE_INCLUDE_DIRS"${CSPARSE_INCLUDE_DIR})
link_directories(${CSparse_LIBRARY_DIRS})
link_libraries(${CSPARSE_LIBRARY})
message("CSparse-libraries ${CSPARSE_LIBRARY}")

file(GLOB_RECURSE HEADERS_G2O
        ${AR_SOURCE_DIR}/tutorial_slam2d/class/*.h
        ${AR_SOURCE_DIR}/tutorial_slam2d/class/*.cpp
)
message("HEADERS_AR = ${HEADERS_G2O}")

ADD_LIBRARY(tutorial_slam2d_library   SHARED  ${HEADERS_G2O})
TARGET_LINK_LIBRARIES(tutorial_slam2d_library  ${G2O_LIBRARIES} )

ADD_EXECUTABLE(tutorial_slam2d   ${AR_SOURCE_DIR}/tutorial_slam2d/tutorial_slam2d.cpp)
TARGET_LINK_LIBRARIES(tutorial_slam2d tutorial_slam2d_library)



##############################################################################################
############################## EKFSLAM + g2o  ################################################
##############################################################################################

include_directories(${AR_SOURCE_DIR}/tutorial_slam2d/class)

add_library(AllLibrary SHARED ${HEADERS_AR} ${SOURCES_AR} ${catkin_LIBRARIES}  ${HEADERS_G2O})
target_link_libraries( AllLibrary ${OpenCV_LIBS} ${catkin_LIBRARIES} )

add_executable(SLAM_main ${AR_SOURCE_DIR}/qrslam/main.cpp )
target_link_libraries(SLAM_main ${OpenCV_LIBS} AllLibrary ${catkin_LIBRARIES} )
