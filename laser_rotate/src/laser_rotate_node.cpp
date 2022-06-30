#include <ros/ros.h>

#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <laser_rotate/AngleOffsetConfig.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher rlaserPub;

double angleOffset;

void dynReconfigureUpdateCallback(laser_rotate::AngleOffsetConfig &config, uint32_t level) {
    angleOffset = config.angle_offset;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Add angleOffset to the msg
    sensor_msgs::LaserScan result;
    result = *msg;
    result.angle_min += (float)angleOffset;
    result.angle_max += (float)angleOffset;
    rlaserPub.publish(result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_rotator");
    ros::NodeHandle node;

    angleOffset = M_PI_2;

    dynamic_reconfigure::Server<laser_rotate::AngleOffsetConfig> server;
    dynamic_reconfigure::Server<laser_rotate::AngleOffsetConfig>::CallbackType f;

    f = boost::bind(&dynReconfigureUpdateCallback, _1, _2);
    server.setCallback(f);

    rlaserPub = node.advertise<sensor_msgs::LaserScan>("scan", 100);
    ros::Subscriber laserSub = node.subscribe("scan_raw", 100, laserCallback);

    ros::spin();

    return 0;
}
