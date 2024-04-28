#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>
#include <tf/transform_broadcaster.h> 

double lat_zero;
double lon_zero;
double alt_zero;
double prev_x = 0.0;
double prev_y = 0.0;
double prev_yaw = 0.0;
bool initial_pose_set = false;
double Xr,Yr,Zr;

ros::Publisher pub;

void GpsToEcef(double lat, double lon, double alt, double& ecef_x, double& ecef_y, double& ecef_z) {
    const double e = 0.0066944782; 
    const double a = 6378137;  
    const double N = a / sqrt(1 - (e * sin(lat) * sin(lat)));
    ecef_x = (N + alt) * cos(lat) * cos(lon);
    ecef_y = (N + alt) * cos(lat) * sin(lon);
    ecef_z = ((1 - e )* N + alt) * sin(lat);
}

void EcefToEnu(double ecef_x, double ecef_y, double ecef_z, double& enu_x, double& enu_y, double& enu_z) {
    double cos_lat = cos(lat_zero);
    double sin_lat = sin(lat_zero);
    double cos_lon = cos(lon_zero);
    double sin_lon = sin(lon_zero);
    double dx = ecef_x - Xr;
    double dy = ecef_y - Yr;
    double dz = ecef_z - Zr;
    enu_x = (-sin_lon * dx) + (cos_lon * dy);
    enu_y = (-sin_lat * cos_lon * dx) + (-sin_lat * sin_lon * dy) + (cos_lat * dz);
    enu_z = (cos_lat * cos_lon * dx) + (cos_lat * sin_lon * dy) + (sin_lat * dz);
}

double ComputeYaw(double x, double y) {
    return atan2(y - prev_y, x - prev_x);
}

void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    double altitude = msg->altitude;
    //ROS_INFO (" GPS %f %f %f ",latitude,longitude,altitude);
    latitude=latitude *M_PI/180;
    longitude=longitude *M_PI/180;
    double ecef_x, ecef_y, ecef_z;
    GpsToEcef(latitude, longitude, altitude, ecef_x, ecef_y, ecef_z);
    //ROS_INFO (" ECEF %f %f %f ",ecef_x,ecef_y,ecef_z);
    double enu_x, enu_y, enu_z;
    EcefToEnu(ecef_x, ecef_y, ecef_z, enu_x, enu_y, enu_z);
    ROS_INFO(" ENU %f %f %f ",enu_x,enu_y,enu_z);
    nav_msgs::Odometry output_msg;
    output_msg.header = msg->header;
    output_msg.pose.pose.position.x = enu_x;
    output_msg.pose.pose.position.y = enu_y;
    output_msg.pose.pose.position.z = enu_z;

    double yaw;
    if (initial_pose_set) {
        yaw = ComputeYaw(enu_x, enu_y);
    } else {
        yaw = 0.0;
        initial_pose_set = true;
    }
    prev_x = enu_x;
    prev_y = enu_y;
    output_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    pub.publish(output_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom_converter");
    ros::NodeHandle n;
    n.getParam("/lat_zero", lat_zero);
    n.getParam("/lon_zero", lon_zero);
    n.getParam("/alt_zero", alt_zero);
    lat_zero=lat_zero*M_PI/180;
    lon_zero=lon_zero*M_PI/180;
    GpsToEcef(lat_zero, lon_zero, alt_zero, Xr, Yr, Zr);
    ros::Subscriber sub = n.subscribe("/fix", 1000, callback);
    pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1000);
    ros::spin(); 
    return 0;
}
