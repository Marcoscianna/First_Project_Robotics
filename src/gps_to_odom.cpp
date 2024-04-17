#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>
#include <tf/transform_broadcaster.h> // Aggiunto l'inclusione di tf

// Variabili globali per memorizzare i valori dei parametri
double lat_zero;
double lon_zero;
double alt_zero;

// Variabili globali per memorizzare la posizione e l'orientamento precedenti
double prev_x = 0.0;
double prev_y = 0.0;
double prev_yaw = 0.0;
bool initial_pose_set = false;
double Xr,Yr,Zr;


// Publisher globale per pubblicare il messaggio di odometria
ros::Publisher pub;

// Funzione per convertire le coordinate GPS in coordinate ECEF
void GpsToEcef(double lat, double lon, double alt, double& ecef_x, double& ecef_y, double& ecef_z) {
    const double e = 0.0066944782; // valore di e approssimato per il WGS84 ellipsoid
    const double a = 6378137;    // raggio equatoriale del WGS84 ellipsoid
    const double N = a / sqrt(1 - (e * sin(lat) * sin(lat)));
    ecef_x = (N + alt) * cos(lat) * cos(lon);
    ecef_y = (N + alt) * cos(lat) * sin(lon);
    ecef_z = ((1 - e )* N + alt) * sin(lat);
}

// Funzione per convertire le coordinate ECEF in coordinate ENU
void EcefToEnu(double ecef_x, double ecef_y, double ecef_z, double& enu_x, double& enu_y, double& enu_z) {
    double cos_lat = cos(lat_zero);
    double sin_lat = sin(lat_zero);
    double cos_lon = cos(lon_zero);
    double sin_lon = sin(lon_zero);

    //ROS_INFO (" GLOBALI %f %f %f ",Xr,Yr,Zr);
    //ROS_INFO (" ECEF %f %f %f ",ecef_x,ecef_y,ecef_z);
    double dx = ecef_x - Xr;
    double dy = ecef_y - Yr;
    double dz = ecef_z - Zr;
    //ROS_INFO (" DIFF %f %f %f ",dx,dy,dz);

    enu_x = (-sin_lon * dx) + (cos_lon * dy);
    enu_y = (-sin_lat * cos_lon * dx) + (-sin_lat * sin_lon * dy) + (cos_lat * dz);
    enu_z = (cos_lat * cos_lon * dx) + (cos_lat * sin_lon * dy) + (sin_lat * dz);
}

// Funzione per calcolare l'orientamento del robot
double ComputeYaw(double x, double y) {
    return atan2(y - prev_y, x - prev_x);
}

void callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    //ROS_INFO (" received %u ", msg->header.seq);
    // Estrai i dati di odometria dal messaggio
    double latitude = msg->latitude;
    double longitude = msg->longitude;
    double altitude = msg->altitude;
    ROS_INFO (" GPS %f %f %f ",latitude,longitude,altitude);
    // Calcola le coordinate ECEF della posizione GPS ricevuta

    latitude=latitude*M_PI/180;
    longitude=longitude*M_PI/180;

    ROS_INFO (" GPS RADIUS %f %f %f ",latitude,longitude,altitude);
    double ecef_x, ecef_y, ecef_z;
    
    GpsToEcef(msg->latitude, msg->longitude, msg->altitude, ecef_x, ecef_y, ecef_z);
    ROS_INFO (" ECEF %f %f %f ",ecef_x,ecef_y,ecef_z);


    // Calcola le coordinate ENU rispetto al punto di riferimento
    double enu_x, enu_y, enu_z;
    EcefToEnu(ecef_x, ecef_y, ecef_z, enu_x, enu_y, enu_z);
    ROS_INFO(" ENU %f %f %f ",enu_x,enu_y,enu_z);
    // Crea un nuovo messaggio di odometria per inviare i dati elaborati
    nav_msgs::Odometry output_msg;
    output_msg.header = msg->header;
    output_msg.pose.pose.position.x = enu_x; // Coordinate ENU rispetto al punto di riferimento
    output_msg.pose.pose.position.y = enu_y;
    output_msg.pose.pose.position.z = enu_z;

    // Calcola l'orientamento del robot
    double yaw;
    if (initial_pose_set) {
        yaw = ComputeYaw(enu_x, enu_y);
    } else {
        // Se è la prima posizione ricevuta, imposta l'orientamento a zero
        yaw = 0.0;
        initial_pose_set = true;
    }
    // Salva la posizione corrente come posizione precedente per il prossimo calcolo
    prev_x = enu_x;
    prev_y = enu_y;

    // Aggiorna l'orientamento del robot nel messaggio di odometria
    output_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    //ROS_INFO ("%f %f %f ",output_msg.pose.pose.position.x,output_msg.pose.pose.position.y,output_msg.pose.pose.position.z);
    // Pubblica il messaggio di odometria elaborato sul topic "/gps_odom"
    // ROS_INFO (" ENU %f %f %f ",output_msg.pose.pose.position.x,output_msg.pose.pose.position.y,output_msg.pose.pose.position.z);
    pub.publish(output_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_to_odom_converter");
    ros::NodeHandle n;
  
    // Leggi i parametri lat_zero, lon_zero, e alt_zero dal file di lancio
    n.getParam("/lat_zero", lat_zero);
    n.getParam("/lon_zero", lon_zero);
    n.getParam("/alt_zero", alt_zero);

    GpsToEcef(lat_zero, lon_zero, alt_zero, Xr, Yr, Zr);
    //ROS_INFO (" GLOBALI %f %f %f ",Xr,Yr,Zr);
    // Crea un subscriber che ascolta i messaggi dal topic "/fix"
    ros::Subscriber sub = n.subscribe("/fix", 1000, callback);

    // Crea il publisher per il topic "/gps_odom"
    
    pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1000);

    ros::spin(); // Mantieni il nodo in esecuzione finché non viene terminato esternamente

    return 0;
}
