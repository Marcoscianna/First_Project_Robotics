#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "first_project/LidarVisualizationConfig.h" // Modifica questa riga

std::string lidar_frame = "wheel_odom"; // Frame predefinito

void reconfigureCallback(first_project::LidarVisualizationConfig &config, uint32_t level) {
    lidar_frame = config.lidar_frame;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Modifica il frame nel messaggio del LIDAR
    sensor_msgs::LaserScan modified_msg = *msg;
    modified_msg.header.frame_id = lidar_frame;

    // Fai qualcosa con il messaggio modificato...
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_visualization");
    ros::NodeHandle nh;

    // Setup del server di configurazione dinamica
    dynamic_reconfigure::Server<first_project::LidarVisualizationConfig> server;
    dynamic_reconfigure::Server<first_project::LidarVisualizationConfig>::CallbackType f;
    f = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(f);

    // Sottoscrizione al topic del LIDAR
    ros::Subscriber sub = nh.subscribe("/scan", 1, scanCallback);

    ros::spin();

    return 0;
}
