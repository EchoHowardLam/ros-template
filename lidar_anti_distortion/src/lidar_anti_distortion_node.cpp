#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_anti_distortion/AntiDistortionConfig.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher rlaserPub;

double distortionOffset;
double distortionFactor;

void dynReconfigureUpdateCallback(lidar_anti_distortion::AntiDistortionConfig &config, uint32_t level) {
    distortionOffset = config.distortion_offset;
    distortionFactor = config.distortion_factor;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Add angleOffset to the msg
    sensor_msgs::LaserScan result;
    result = *msg;
    for (size_t i = 0; i < result.ranges.size(); i++)
    {
        auto range = result.ranges[i];
        if (result.range_min <= range && range <= result.range_max)
            result.ranges[i] = range * distortionFactor + distortionOffset;
    }
    rlaserPub.publish(result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_distortion_corrector");
    ros::NodeHandle node;

    distortionOffset = 0.0;
    distortionFactor = 1.0;

    dynamic_reconfigure::Server<lidar_anti_distortion::AntiDistortionConfig> server;
    dynamic_reconfigure::Server<lidar_anti_distortion::AntiDistortionConfig>::CallbackType f;

    f = boost::bind(&dynReconfigureUpdateCallback, _1, _2);
    server.setCallback(f);

    rlaserPub = node.advertise<sensor_msgs::LaserScan>("base_scan_rect", 10);
    ros::Subscriber laserSub = node.subscribe("base_scan_raw", 10, laserCallback);

    ros::spin();

    return 0;
}
