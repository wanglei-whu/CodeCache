add_library(lidar_inertial_odometry
            lidar_inertial_odometry.cpp)

add_subdirectory(fusion_filter)
add_subdirectory(lidar_map_online)
add_subdirectory(inertial_navigation)
add_subdirectory(lidar_odometry)
