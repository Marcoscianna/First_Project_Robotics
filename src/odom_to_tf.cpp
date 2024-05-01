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
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
        tf::Quaternion quaternion;
        quaternion.setRPY(0, 0, msg->pose.pose.orientation.w);
        transform.setRotation(quaternion);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, root_frame_, child_frame_));
}


public:
    OdomToTFNode() {
        nh_ = ros::NodeHandle("~");
        nh_.getParam("root_frame", root_frame_);
        nh_.getParam("child_frame", child_frame_);
        sub_ = nh_.subscribe("/input_odom", 1000, &OdomToTFNode::odomCallback, this);
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
