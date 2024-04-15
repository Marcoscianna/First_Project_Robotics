#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomToTFNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string root_frame_;
    std::string child_frame_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Calcola la trasformazione e pubblicala utilizzando il broadcaster tf
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = root_frame_;
        transformStamped.child_frame_id = child_frame_;
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;
        transformStamped.transform.rotation = msg->pose.pose.orientation;

        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, msg->pose.pose.orientation.z);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();

        tf_broadcaster_.sendTransform(transformStamped);
    }

public:
    OdomToTFNode() {
        nh_ = ros::NodeHandle("~");
        nh_.getParam("root_frame", root_frame_);
        nh_.getParam("child_frame", child_frame_);
        sub_ = nh_.subscribe("/odom", 1000, &OdomToTFNode::odomCallback, this);
    }

    void run() {
        ros::spin();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_tf");

    OdomToTFNode node;
    node.run();

    return 0;
}
