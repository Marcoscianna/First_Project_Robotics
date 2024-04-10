#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

class LidarVisualizationNode {

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string lidar_frame_;

    // Callback per il messaggio LaserScan
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Qui puoi gestire i dati del messaggio LaserScan
        // Ad esempio, puoi modificare il frame_id e pubblicare il messaggio modificato
        sensor_msgs::LaserScan modified_scan = *msg;
        modified_scan.header.frame_id = lidar_frame_;
        // Qui puoi pubblicare il messaggio modificato o eseguire altre operazioni necessarie
    }

    // Callback per la riconfigurazione dinamica
    void callback(first_project::parametersConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure Request: %s %s", 
                config.gps_odom.c_str(), 
                config.wheel_odom.c_str());         
        ROS_INFO ("%d",level);

        if (!config.gps_odom.empty() && !config.wheel_odom.empty()) {
            ROS_WARN("Entrambi i frame sono selezionati. Seleziona solo uno.");
        } else if (!config.gps_odom.empty()) {
            lidar_frame_ = "gps_odom";
        } else if (!config.wheel_odom.empty()) {
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

        // Sottoscrizione al topic "/os_cloud_node/points"
        sub_ = nh_.subscribe("/os_cloud_node/points",1000, &LidarVisualizationNode::laserScanCallback, this);

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
