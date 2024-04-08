#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> 
#include <dynamic_reconfigure/server.h>
#include <first_project/LidarFrameConfig.h>

class LidarVisualizationNode {
public:
    LidarVisualizationNode() {
        ros::NodeHandle nh("~");

        nh.param<std::string>("default_frame", frame_id_, "world");
        lidar_frame_ = frame_id_;

        lidar_sub_ = nh.subscribe("/scan", 1, &LidarVisualizationNode::lidarCallback, this);
        dynamic_reconfigure_server_.setCallback(boost::bind(&LidarVisualizationNode::dynamicReconfigureCallback, this, _1, _2));
    }

private:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) { 
        sensor_msgs::LaserScan lidar_msg = *msg; 
        lidar_msg.header.frame_id = lidar_frame_;
    }

    void dynamicReconfigureCallback(first_project::LidarFrameConfig& config, uint32_t level) {
        lidar_frame_ = config.lidar_frame;
    }

    ros::Subscriber lidar_sub_;
    dynamic_reconfigure::Server<first_project::LidarFrameConfig> dynamic_reconfigure_server_;
    std::string frame_id_;
    std::string lidar_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_visualization_node");
    LidarVisualizationNode node;
    ros::spin(); 
    return 0;
}
