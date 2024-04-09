#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <cmath>

// Variabili globali per memorizzare i valori dei parametri
double lat_zero;
double lon_zero;
double alt_zero;

// Funzione per convertire le coordinate GPS in coordinate ECEF
void GpsToEcef(double lat, double lon, double alt, double& ecef_x, double& ecef_y, double& ecef_z){
    double e = 0.08181919; // valore di e approssimato per il WGS84 ellipsoid
    double a = 6378137;    // raggio equatoriale del WGS84 ellipsoid
    double N;

    N = a / sqrt(1 - (e * e * sin(lat) * sin(lat)));
    ecef_x = (N + alt) * cos(lat) * cos(lon);
    ecef_y = (N + alt) * cos(lat) * sin(lon);
    ecef_z = ((1 - e * e) * N + alt) * sin(lat);
}

void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    // Estrai i dati di odometria dal messaggio
    double latitude = msg->latitude -lat_zero;
    double longitude = msg->longitude -lon_zero;
    double altitude = msg->altitude - alt_zero;

    // Stampa i dati di odometria
    //ROS_INFO("Received GPS data: latitude=%f, longitude=%f, altitude=%f", latitude, longitude, altitude);

    // Calcola le coordinate ECEF della posizione GPS ricevuta
    double ecef_x, ecef_y, ecef_z;
    GpsToEcef(latitude, longitude, altitude, ecef_x, ecef_y, ecef_z);

    // Calcola le coordinate ECEF dell'origine
    double origin_ecef_x, origin_ecef_y, origin_ecef_z;
    

    // Calcola le coordinate NED rispetto all'origine (0,0,0)
    double ned_x = ecef_x ;
    double ned_y = ecef_y; 
    double ned_z = ecef_z; 

   //ROS_INFO("Received NED data: latitude=%f, longitude=%f, altitude=%f", ned_x, ned_y, ned_z);
    // Crea un nuovo messaggio di odometria per inviare i dati elaborati
    nav_msgs::Odometry output_msg;
    output_msg.header = msg->header;
    output_msg.pose.pose.position.x = ned_x; // Coordinate NED rispetto all'origine (0,0,0)
    output_msg.pose.pose.position.y = ned_y;
    output_msg.pose.pose.position.z = ned_z;

    // Pubblica il messaggio di odometria elaborato sul topic "/gps_odom"
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
    pub.publish(output_msg);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "gps_to_odom_converter");
    ros::NodeHandle n;
    // Leggi i parametri lat_zero, lon_zero, e alt_zero
    n.param("lat_zero", lat_zero, 0.0);
    n.param("lon_zero", lon_zero, 0.0);
    n.param("alt_zero", alt_zero, 0.0);
    // Crea un subscriber che ascolta i messaggi dal topic "/fix"
    ros::Subscriber sub = n.subscribe("/fix", 1, callback);
    ros::spin(); // Mantieni il nodo in esecuzione finché non viene terminato esternamente
    return 0;
}
