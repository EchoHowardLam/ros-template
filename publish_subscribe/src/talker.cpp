#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::Publisher talkerPub = n.advertise<std_msgs::String>("chatter", 100);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "Hello";

        talkerPub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}