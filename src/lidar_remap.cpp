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

    // Callback per la riconfigurazione dinamica
    void callback(first_project::parametersConfig &config, uint32_t level) {
        ROS_INFO("Reconfigure Request: %s %s", 
                config.gps_odom.c_str(), 
                config.wheel_odom.c_str());         
        ROS_INFO("%d", level);

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

    // Callback per il messaggio della nuvola di punti
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Creazione del messaggio di nuvola di punti da pubblicare
        sensor_msgs::PointCloud2 laser_scan = *msg;

        // Aggiornamento dell'header con il nuovo frame ID
        laser_scan.header.frame_id = lidar_frame_;

        // Pubblicazione del messaggio aggiornato
        pub_.publish(laser_scan);
    }

public:
    LidarVisualizationNode() {
        nh_ = ros::NodeHandle("~");

        // Inizializzazione del frame_id predefinito
        lidar_frame_ = "default_frame"; 

        // Inizializzazione del publisher per pubblicare i dati del sensore laser modificati
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1000);
    }

    void run() {
        dynamic_reconfigure::Server<first_project::parametersConfig> server;
        dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;

        // Associa la callback al server di riconfigurazione dinamica
        f = boost::bind(&LidarVisualizationNode::callback, this, _1, _2);
        server.setCallback(f);

        // Sottoscrizione al topic "/os_cloud_node/points"
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
