#include <ros/ros.h>

#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <auto_locate/AutoLocateConfig.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double lidarOffset = 0.0;
double robotRadius = 1000.0;
double turningSpeed = 0.1;
double turningAngle = 0.0;

bool hasScanData = false;
sensor_msgs::LaserScan lastScanData;
bool hasImuData = false;
sensor_msgs::Imu lastImuData;
bool willCollideDefined = false;
bool willCollide;

bool performAutoLocate = false;
bool initYawDefined;
double initYaw;
int rotateStage = 0;

double extractYawFromImu(const sensor_msgs::Imu &msg)
{
    double r, p, y;
    tf2::Quaternion quat;
    tf2::fromMsg(msg.orientation, quat);
    tf2::Matrix3x3(quat).getRPY(r, p, y);
    return y;
}
// Expected yaw1 and yaw2 to be normalized, i.e. in range [-pi, pi]
// Get the signed diff needed to move from yaw1 to yaw2
// Always choose the smaller angle change among clockwise and counter-clockwise rotation
double getYawDifference(double yaw1, double yaw2) {
    if (yaw2 < yaw1) yaw2 += 2 * M_PI; // Make sure diff is always positive in the next line
    double diff = yaw2 - yaw1;
    if (diff > M_PI) diff -= 2 * M_PI; // Take the smaller angle between CC and CCW
    return diff;
}

void autoLocateConfigUpdateCallback(auto_locate::AutoLocateConfig &config, uint32_t level)
{
    lidarOffset = config.lidar_offset;
    robotRadius = config.robot_radius;
    turningSpeed = config.turning_speed;
    turningAngle = config.turning_angle;
    willCollideDefined = false;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    { hasScanData = true; lastScanData = *msg; willCollideDefined = false; }
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    { hasImuData = true; lastImuData = *msg; }

void doAutoLocateCallback(const std_msgs::Empty::ConstPtr& msg)
    { performAutoLocate = true; initYawDefined = false; rotateStage = 0; }

// Update variable willCollide
void updateCollisionInfo()
{
    //if (willCollideDefined) return;

    willCollide = false;
    const auto &msg = lastScanData;
    for (std::size_t i = 0; i < msg.ranges.size(); ++i)
    {
        float range = msg.ranges[i];
        if (msg.range_min <= range && range <= msg.range_max)
        {
            if (range + lidarOffset < robotRadius)
            {
                willCollide = true;
                break;
            }
        }
    }

    willCollideDefined = true;
}

bool rotate(geometry_msgs::Twist &twist)
{
    twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
    twist.angular.x = twist.angular.y = twist.angular.z = 0.0;

    // Wait until last scan & imu data is recent
    if (!hasScanData ||
        ros::Time::now() - lastScanData.header.stamp > ros::Duration(0.5)
        ) {
        ROS_WARN("No recent scan data");
        return false;
    }
    if (!hasImuData ||
        ros::Time::now() - lastImuData.header.stamp > ros::Duration(0.5)
        ) {
        ROS_WARN("No recent imu data");
        return false;
    }

    // Detect potential collisions around the robot
    //if (!willCollideDefined)
        updateCollisionInfo();
    if (willCollide)
    {
        ROS_WARN("Obstacle is too close to robot for in place rotation. Robot r = %f", robotRadius);
        return false;
    }

    if (!initYaw)
    {
        initYawDefined = true;
        initYaw = extractYawFromImu(lastImuData);
    }

    double curYaw = extractYawFromImu(lastImuData);
    ROS_INFO("Rotating according to stage %d", rotateStage);
    switch (rotateStage)
    {
    case 0:
        if (getYawDifference(initYaw, curYaw) < turningAngle)
            twist.angular.z = turningSpeed;
        else
            rotateStage = 1;
        break;
    case 1:
        if (getYawDifference(initYaw, curYaw) > -turningAngle)
            twist.angular.z = -turningSpeed;
        else
            rotateStage = 2;
        break;
    case 2:
        if (getYawDifference(initYaw, curYaw) < 0.0)
            twist.angular.z = turningSpeed;
        else
            rotateStage = 3;
        break;
    default:
        break;
    }
    if (rotateStage > 2)
    {
        ROS_INFO("Auto rotation completed");
        return true;
    }

    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_locator");
    ros::NodeHandle node;

    dynamic_reconfigure::Server<auto_locate::AutoLocateConfig> server;
    dynamic_reconfigure::Server<auto_locate::AutoLocateConfig>::CallbackType f;

    f = boost::bind(&autoLocateConfigUpdateCallback, _1, _2);
    server.setCallback(f);

    ros::Publisher cmdvelPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    ros::Subscriber laserSub = node.subscribe("scan", 10, laserCallback);
    ros::Subscriber imuSub = node.subscribe("imu", 100, imuCallback);
    ros::Subscriber doAutoLocateSub = node.subscribe("perform_auto_locate", 100, doAutoLocateCallback);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (performAutoLocate)
        {
            geometry_msgs::Twist twist;
            if (rotate(twist))
                performAutoLocate = false;
            cmdvelPub.publish(twist);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
