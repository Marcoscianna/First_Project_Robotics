#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include <cmath>


// Variabili globali per memorizzare i valori dei parametri
double lat_zero;
double lon_zero;
double alt_zero;


// Funzione per convertire le coordinate GPS in coordinate ECEF
void GpsToEcef(double lat, double lon, double alt, double& ecef_x, double& ecef_y, double& ecef_z){
    double e = 0.00669437999014;
    double a = 6378137;
    double N;


    N=a/sqrt(1-(e*e*sin(lat)*sin(lat)));


    ecef_x=(N+alt)*cos(lat)*cos(lon);
    ecef_y=(N+alt)*cos(lat)*sin(lon);
    ecef_z=((1-e*e)*N+alt)*sin(lat);
}




// Funzione per convertire le coordinate ECEF in coordinate NED rispetto all'origine (0,0,0)
void ecefToNED(double ecef_x, double ecef_y, double ecef_z, double& ned_x, double& ned_y, double& ned_z) {
   // Calcola le differenze di posizione rispetto all'origine (0,0,0) in coordinate ECEF
   ned_x = ecef_x;
   ned_y = ecef_y;
   ned_z = ecef_z;
}


void Callback(const nav_msgs::Odometry::ConstPtr& msg) {
   // Estrai i dati di odometria dal messaggio
   double x = msg->pose.pose.position.x;
   double y = msg->pose.pose.position.y;
   double z = msg->pose.pose.position.z;


   // Stampa i dati di odometria
   ROS_INFO("Received odometry data: x=%f, y=%f, z=%f", x, y, z);
//Calcola le coordinate ECEF
    double ecef_x, ecef_y, ecef_z;
    GpsToEcef(x, y, z, ecef_x, ecef_y, ecef_z);




   // Calcola le coordinate NED rispetto all'origine (0,0,0)
   double ned_x, ned_y, ned_z;
   ecefToNED(ecef_x, ecef_y, ecef_z, ned_x, ned_y, ned_z);


   // Aggiungi le coordinate di riferimento
   ned_x -= lon_zero;
   ned_y -= lat_zero;
   ned_z -= alt_zero;


   // Crea un nuovo messaggio di odometria per inviare i dati elaborati
   nav_msgs::Odometry output_msg;
   output_msg.header = msg->header;
   output_msg.pose.pose.position.x = ned_x; // Coordinate NED rispetto all'origine (0,0,0)
   output_msg.pose.pose.position.y = ned_y;
   output_msg.pose.pose.position.z = ned_z;


   // Pubblica il messaggio di odometria elaborato sul topic input_odom
   ros::NodeHandle n;
   ros::Publisher pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
   pub.publish(output_msg);
}


int main(int argc, char **argv){
   ros::init(argc, argv, "talker");
   ros::NodeHandle n;


   // Leggi i parametri lat_zero, lon_zero, e alt_zero
   n.param("lat_zero", lat_zero, 0.0);
   n.param("lon_zero", lon_zero, 0.0);
   n.param("alt_zero", alt_zero, 0.0);


   // Crea un subscriber che ascolta i messaggi dal topic "/gps_odom"
   ros::Subscriber sub = n.subscribe("/fix", 1, Callback);


   ros::spin(); // Mantieni il nodo in esecuzione finché non viene terminato esternamente


   return 0;
}