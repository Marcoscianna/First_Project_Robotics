#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/parametersConfig.h>

class LidarVisualizationNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string lidar_frame_;

    // Callback per il messaggio PointCloud2
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Qui puoi convertire i dati della nuvola di punti in dati di scansione laser

        // Supponiamo che tu voglia convertire la distanza media dei punti in una scansione laser
        sensor_msgs::LaserScan laser_scan;
        laser_scan.header = msg->header;
        laser_scan.angle_min = -M_PI / 2;  // Angolo minimo
        laser_scan.angle_max = M_PI / 2;   // Angolo massimo
        laser_scan.angle_increment = M_PI / msg->width;  // Incremento dell'angolo
        laser_scan.time_increment = 0.0;  // Incremento del tempo
        laser_scan.scan_time = 0.1;  // Tempo di scansione
        laser_scan.range_min = 0.0;  // Distanza minima rilevabile
        laser_scan.range_max = 10.0;  // Distanza massima rilevabile

        // Calcola la distanza media dei punti e assegnala alla scansione laser
        std::vector<float> ranges;
        for (size_t i = 0; i < msg->width; ++i) {
            float range = sqrt(pow(msg->data[i * msg->point_step], 2) + 
                               pow(msg->data[i * msg->point_step + 1], 2) + 
                               pow(msg->data[i * msg->point_step + 2], 2));
            ranges.push_back(range);
        }
        laser_scan.ranges = ranges;

        // Imposta il frame ID
        laser_scan.header.frame_id = lidar_frame_;

        // Pubblica il messaggio di scansione laser
        pub_.publish(laser_scan);
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

        // Inizializzazione del publisher per pubblicare i dati del sensore laser modificati
        pub_ = nh_.advertise<sensor_msgs::LaserScan>("/modified_scan", 1000);
    }

    void run() {
        dynamic_reconfigure::Server<first_project::parametersConfig> server;
        dynamic_reconfigure::Server<first_project::parametersConfig>::CallbackType f;

        // Associa la callback al server di riconfigurazione dinamica
        f = boost::bind(&LidarVisualizationNode::callback, this, _1, _2);
        server.setCallback(f);

        // Sottoscrizione al topic "/os_cloud_node/points"
        sub_ = nh_.subscribe("/os_cloud_node/points", 1000, &LidarVisualizationNode::pointCloudCallback, this);

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
