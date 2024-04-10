#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h> 
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

class LidarVisualizationNode {

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string lidar_frame_;

    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) { 
        sensor_msgs::LaserScan lidar_msg = *msg; 
        lidar_msg.header.frame_id = lidar_frame_;
    }

    void callback(first_project::parametersConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure Request: gps_odom=%s, wheel_odom=%s", 
                config.gps_odom ? "true" : "false", 
                config.wheel_odom ? "true" : "false");         

        // Modifica del frame_id dell'intestazione in base al valore dei parametri
        if (config.gps_odom) {
            lidar_frame_ = "gps_odom";
        } else if (config.wheel_odom) {
            lidar_frame_ = "wheel_odom";
        } else {
            ROS_WARN("Nessun frame selezionato.");
        }
    }

public:
    LidarVisualizationNode() {
        nh_ = ros::NodeHandle("~");

        // Inizializzazione del frame_id predefinito
        lidar_frame_ = "default_frame"; 
    }

    void run() {
        dynamic_reconfigure::Server<first_project::parametersConfig> server;
        dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;

        // Associa la callback al server di riconfigurazione dinamica
        f = boost::bind(&LidarVisualizationNode::callback, this, _1, _2);
        server.setCallback(f);

        // Sottoscrizione al topic "/scan"
        sub_ = nh_.subscribe("/scan", 1000, &LidarVisualizationNode::lidarCallback, this);

        ROS_INFO("Nodo avviato");
        ros::spin();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_visualization");

    LidarVisualizationNode node;
    node.run();

    return 0;
}
