#include <ros/ros.h>

#include <random>

#include <sensor_msgs/LaserScan.h>

ros::Publisher rlaserPub;

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    static double curDistortion = 0.0;

    static std::random_device rd;
    static std::mt19937 mt(rd());
    static std::uniform_real_distribution<double> dist(-0.02, 0.02);

    curDistortion += dist(mt);
    curDistortion = std::max(-0.01, std::min(curDistortion, 0.2));

    // Add angleOffset to the msg
    sensor_msgs::LaserScan result;
    result = *msg;
    for (size_t i = 0; i < result.ranges.size(); i++)
    {
        auto range = result.ranges[i];
        if (result.range_min <= range && range <= result.range_max)
            result.ranges[i] = range + curDistortion;
    }
    rlaserPub.publish(result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_pseudo_distortion_generator");
    ros::NodeHandle node;

    rlaserPub = node.advertise<sensor_msgs::LaserScan>("/robot1/scan_distorted", 100);
    ros::Subscriber laserSub = node.subscribe("/robot1/scan", 100, laserCallback);

    ros::spin();

    return 0;
}
