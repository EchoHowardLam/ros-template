#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::Transform geometryMsgPose_to_geometryMsgTF(geometry_msgs::Pose pose)
{
    geometry_msgs::Transform tf;
    tf.translation.x = pose.position.x;
    tf.translation.y = pose.position.y;
    tf.translation.z = pose.position.z;
    tf.rotation.x = pose.orientation.x;
    tf.rotation.y = pose.orientation.y;
    tf.rotation.z = pose.orientation.z;
    tf.rotation.w = pose.orientation.w;
    return tf;
}

int main(int argc, char **argv)
{
    // This program tests the transformation of a pose into another coordinate system

    ros::init(argc, argv, "node");
    ros::NodeHandle n;

    // Goal: Transform inputPose from frame A to frame B
    geometry_msgs::Pose inputPose;
    inputPose.position.x = 1.0;
    inputPose.position.y = 0.0;
    inputPose.position.z = 0.0;
    inputPose.orientation.x = 0.0;
    inputPose.orientation.y = 0.0;
    inputPose.orientation.z = 0.0;
    inputPose.orientation.w = 1.0;

    // The transformation from frame A to frame B
    geometry_msgs::Transform inputTFMsg;
    inputTFMsg.translation.x = 5.0;
    inputTFMsg.translation.y = 5.0;
    inputTFMsg.translation.z = 0.0;
    inputTFMsg.rotation.x = 0.0;
    inputTFMsg.rotation.y = 0.0;
    inputTFMsg.rotation.z = 0.0;
    inputTFMsg.rotation.w = 1.0;

    ros::Rate loop_rate(0.1);
    while (ros::ok())
    {
        tf2::Transform inputPoseTF;
        tf2::fromMsg(geometryMsgPose_to_geometryMsgTF(inputPose), inputPoseTF);

        tf2::Transform inputTFTF;
        tf2::fromMsg(inputTFMsg, inputTFTF);

        ROS_INFO_STREAM(tf2::toMsg(inputPoseTF * inputTFTF.inverse()));

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

