#include "ros/ros.h"

#include <geometry_msgs/TransformStamped.h>

#include <tf/tf.h>                 // tf::resolve
#include <tf/transform_listener.h> // tf::getPrefixParam

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

std::string frameA = "A";
std::string frameB = "B";

void listen_tf()
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped tf_A_to_B;
    try {
        tf_A_to_B = tfBuffer.lookupTransform(
            frameA,
            frameB,
            ros::Time(0)
        );
        ROS_INFO("Yes");
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Not receiving TF. %s", ex.what());
    }
}

void send_tf()
{
    static tf2_ros::TransformBroadcaster bc;

    geometry_msgs::TransformStamped tf_A_to_B;
    tf_A_to_B.header.stamp = ros::Time::now();
    tf_A_to_B.header.frame_id = frameA;
    tf_A_to_B.child_frame_id = frameB;
    tf_A_to_B.transform.rotation.w = 1.0;
    bc.sendTransform(tf_A_to_B);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_test");
    ros::NodeHandle node;

    ros::NodeHandle private_node("~");
    std::string tf_prefix = tf::getPrefixParam(node);
    std::string action = "";
    if (private_node.hasParam("action"))
        private_node.getParam("action", action);

    frameA = tf::resolve(tf_prefix, frameA);
    frameB = tf::resolve(tf_prefix, frameB);

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        if (action == "get")
            listen_tf();
        else if (action == "set")
            send_tf();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
