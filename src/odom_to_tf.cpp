#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>

class OdomToTFNode {
public:
    OdomToTFNode() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Retrieve node parameters
        //string root_frame;
        //string child_frame;
        nh.param<std::string>("root_frame", root_frame, "world");
        nh.param<std::string>("child_frame", child_frame, "wheel_odom");

        ROS_INFO("Root frame: %s, Child frame: %s", root_frame.c_str(), child_frame.c_str());

        // Subscribe to the input odometry topic
        odom_sub_ = nh.subscribe("input_odom", 1, &OdomToTFNode::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Convert odometry message to tf message
        //ROS_INFO("Messaggio ricevuto: %s", msg->header.frame_id.c_str());

        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header = msg->header;
        transformStamped.child_frame_id = child_frame;
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, msg->pose.pose.orientation.z);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();

        // Publish the transform
        tf_broadcaster_.sendTransform(transformStamped);
    }

private:
    ros::Subscriber odom_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string root_frame;
    std::string child_frame;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf_node");
    OdomToTFNode node;
    ros::spin();
    return 0;
}
