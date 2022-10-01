#include <ros/ros.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>

ros::Publisher *dirPubPtr;

void quat2euler(const sensor_msgs::Imu& msg) {
    tf2::Quaternion quat;
    double r, p, y;
    tf2::fromMsg(msg.orientation, quat);
    tf2::Matrix3x3 m(quat);
    m.getRPY(r, p, y);

    geometry_msgs::Vector3 ret;
    ret.x = r;
    ret.y = p;
    ret.z = y;

    dirPubPtr->publish(ret);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "quat_to_imu_node");
    ros::NodeHandle private_node("~");
    ros::NodeHandle node;

    ros::Subscriber imuSub_;

    ros::Publisher dirPub = node.advertise<geometry_msgs::Vector3>("/orientation", 100);
    dirPubPtr = &dirPub;

    imuSub_ = node.subscribe("/imu", 1, &quat2euler);
  
    ros::spin();
    return 0;
}
