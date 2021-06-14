cmake_minimum_required(VERSION 3.10)
project(zed2_client)
set(CMAKE_BUILD_TYPE release)
set(CMAKE_CXX_STANDARD 14)


find_package( OpenCV REQUIRED )

if(OpenCV_VERSION VERSION_LESS "4.0")
    add_definitions(-DOPENCV_VERSION_3_ZED)
else()
    add_definitions(-DOPENCV_VERSION_4_ZED)
endif()


find_package(catkin REQUIRED
        roscpp  pcl_ros tf
        visualization_msgs zed_interfaces
        compressed_depth_image_transport
        pcl_conversions
        message_filters
        image_transport
        image_geometry
        cv_bridge
        )


catkin_package(
        INCLUDE_DIRS include
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(test_subscriber test/DepthCompressedSubscriber.cpp  src/Utils.cpp  )
target_link_libraries(test_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable(zed2_client_node main.cpp src/Client.cpp  src/Utils.cpp  src/Client.cpp )
target_link_libraries(zed2_client_node   ${catkin_LIBRARIES}  )

add_executable(clock_publisher  test/ClockPublisher.cpp )
target_link_libraries(clock_publisher   ${catkin_LIBRARIES}  )

