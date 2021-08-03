#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher scanPub = n.advertise<sensor_msgs::LaserScan>("/scan", 1000);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        sensor_msgs::LaserScan msg;
        std::vector<float> emptyVec;

        msg.header.frame_id = "/laser_scan";
        msg.angle_min = 0.0f;
        msg.angle_max = 0.0f;
        msg.angle_increment = 0.0f;
        msg.time_increment = 0.0f;
        msg.scan_time = 0.0f;
        msg.range_min = 0.0f;
        msg.range_max = 0.0f;
        msg.ranges = emptyVec;
        msg.intensities = emptyVec;

        scanPub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
