#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

class LidarVisualizationNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string lidar_frame_;

    void callback(first_project::parametersConfig &config, uint32_t level) {      
        lidar_frame_ = config.lidar_frame_;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        sensor_msgs::PointCloud2 laser_scan = *msg;     
        laser_scan.header.frame_id = lidar_frame_;
        laser_scan.header.stamp = msg->header.stamp;
        pub_.publish(laser_scan);
    }

public:
    LidarVisualizationNode() {
        nh_ = ros::NodeHandle("~");
        lidar_frame_ = "default_frame"; 
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1000);
    }

    void run() {
        dynamic_reconfigure::Server<first_project::parametersConfig> server;
        dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;
        f = boost::bind(&LidarVisualizationNode::callback, this, _1, _2);
        server.setCallback(f);
        sub_ = nh_.subscribe("/os_cloud_node/points", 1000, &LidarVisualizationNode::pointCloudCallback, this);
        ros::spin();
    }
}; 

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_visualization_node");
    LidarVisualizationNode node;
    node.run();
    return 0;
}