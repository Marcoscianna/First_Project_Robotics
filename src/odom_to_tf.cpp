#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class OdomToTFNode {
public:
    OdomToTFNode() {
        ros::NodeHandle nh("~");

        // Retrieve node parameters
        nh.param<std::string>("root_frame", root_frame_, "world");
        nh.param<std::string>("child_frame", child_frame_, "world");

        // Subscribe to the input odometry topic
        odom_sub_ = nh.subscribe("input_odom", 1, &OdomToTFNode::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Convert odometry message to tf message
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header = msg->header;
        transformStamped.child_frame_id = child_frame_;
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;
        transformStamped.transform.rotation = msg->pose.pose.orientation;

        // Publish the transform
        tf_broadcaster_.sendTransform(transformStamped);
    }

private:
    ros::Subscriber odom_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string root_frame_;
    std::string child_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf_node");
    OdomToTFNode node;
    ros::spin();
    return 0;
}
