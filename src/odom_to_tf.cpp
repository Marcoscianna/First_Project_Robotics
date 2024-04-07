#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
//#include <nav_msgs/Odometry.h>




int main(int argc, char** argv){
   // Initialize the ROS node
   ros::init(argc, argv, "odom_to_tf");


   // Create a ROS node handle
   ros::NodeHandle node;


   // Create a TransformListener object that will listen to tf data
   tf::TransformListener listener;


   // Set the rate at which we want to check the transformation
   ros::Rate rate(10.0);


   while (node.ok()){
       tf::StampedTransform transform;


       try{
           // Wait for up to 1 second for the transform to become available
           listener.waitForTransform("/input_odom", "/root_frame", ros::Time(0), ros::Duration(1.0));


           // Look up the transformation from "world" to "FRleg"
           listener.lookupTransform("/input_odom", "/root_frame", ros::Time(0), transform);
           listener.lookupTransform("/input_odom", "/child_frame", ros::Time(0), transform);


       }
       catch (tf::TransformException &ex) {
           // If there is an exception print the error message
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
           continue;
       }




       // Print transformation (only pose, but you can get the orientation)
       ROS_INFO("Translation: x=%f, y=%f, z=%f",
                transform.getOrigin().x(),
                transform.getOrigin().y(),
                transform.getOrigin().z());


       // Sleep
       rate.sleep();
   }


   return 0;
}