link_directories(${LIBRARY_OUTPUT_PATH})
include_directories(include ${catkin_INCLUDE_DIRS})
# examples
#add_executable(qrcodelocalize ${AR_SOURCE_DIR}/qrcodelocalizer/main.cpp)
#target_link_libraries(qrcodelocalize ${OpenCV_LIBS} ARToolKitPlus ${catkin_LIBRARIES} )

add_executable(camera_calibration ${AR_SOURCE_DIR}/camera_calibration/calibrate.cpp )
target_link_libraries(camera_calibration ${OpenCV_LIBS} ARToolKitPlus ${catkin_LIBRARIES}  )

add_executable(arlisten ${AR_SOURCE_DIR}/qrcodelocalizer/arlisten.cpp )
target_link_libraries(arlisten ${OpenCV_LIBS} ARToolKitPlus ${catkin_LIBRARIES}  )


add_executable(qrslam ${AR_SOURCE_DIR}/qrslam/qrslam.cpp )
target_link_libraries(qrslam ${OpenCV_LIBS} ARToolKitPlus ${catkin_LIBRARIES}  )


add_executable(odompub ${AR_SOURCE_DIR}/qrslam/pub/odompub.cpp )
target_link_libraries(odompub    ${catkin_LIBRARIES}  )

add_executable(odom_listen ${AR_SOURCE_DIR}/qrslam/pub/odom_listen.cpp )
target_link_libraries(odom_listen    ${catkin_LIBRARIES}  )



#add_executable(main ${AR_SOURCE_DIR}/qrslam/main.cpp )
#target_link_libraries(main ${OpenCV_LIBS} ARToolKitPlus ${catkin_LIBRARIES}  )


add_executable(image_convert ${AR_SOURCE_DIR}/image_convert/image_converter.cpp )
target_link_libraries(image_convert ${OpenCV_LIBS} ARToolKitPlus ${catkin_LIBRARIES}  )
