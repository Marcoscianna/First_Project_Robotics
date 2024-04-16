cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Trova i pacchetti necessari di ROS
find_package(catkin REQUIRED COMPONENTS
   roscpp nav_msgs tf
   roscpp
   std_msgs
   dynamic_reconfigure)

generate_dynamic_reconfigure_options(
    cfg/parameters.cfg
  )

## Specifica i pacchetti di cui il tuo pacchetto dipende direttamente
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs)

## Specifica gli include e i link directories
include_directories(include ${catkin_INCLUDE_DIRS})

## Definisci l'eseguibile del nodo ROS
add_executable(gps_to_odom src/gps_to_odom.cpp)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(lidar_remap src/lidar_remap.cpp)
add_dependencies(lidar_remap ${PROJECT_NAME}_gencfg)
target_link_libraries(lidar_remap ${catkin_LIBRARIES})

##add_executable(lidar_visualization src/lidar_visualization.cpp)
##target_link_libraries(lidar_visualization ${catkin_LIBRARIES})

## Aggiungi l'esecuzione di rviz
install(PROGRAMS
  scripts/run_rviz
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS
  scripts/rqt_reconfigure
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
