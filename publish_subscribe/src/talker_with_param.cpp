#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");


    /**
     * Recommended method to get tf from environment
     *     std::string tfPrefix = "";
     *     tfPrefix = tf::getPrefixParam(node);
     *     if (tfPrefix == "")
     *         tfPrefix = tf::getPrefixParam(private_node);
     *
     *     auto fullFrameName = tf::resolve(body.getTfPrefix(), "frame_name");
     */


    /**
     * Public parameters are input via:
     * Launch file:
     *     <!-- This method is not tested yet -->
     *     <group ns="namespace_name">
     *         <param name="header" value="He" />
     *         ...
     *     </group>
     * Command line:
     *     # Note that header is saved in server and is therefore persistent across calls
     *     # Public parameters will appear in `rosparam list` as
     *     # /header
     *     rosparam set header "He"
     *     rosrun package node
     */
    std::string messageHeader = "Hello";
    if (node.hasParam("header"))
        node.getParam("header", messageHeader);

    /**
     * Private parameters are input via:
     * Launch file:
     *     <node pkg="april_docking" type="docking_server" name="docking_server" output="screen">
     *         <param name="footer" value="Word"/>
     *         <rosparam param="array_param">[-0.1, 0.0, 0.1]</rosparam>
     *     </node>
     * Command line:
     *     # Note that _footer is saved in server and is therefore persistent across calls
     *     # Private parameters will appear in `rosparam list` as
     *     # /node_namespace/footer
     *     rosrun package node _footer:="Word"
     */
    std::string messageFooter = "World";
    if (private_node.hasParam("footer"))
        private_node.getParam("footer", messageFooter);

    ros::Publisher talkerPub = node.advertise<std_msgs::String>("chatter", 100);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = messageHeader + " " + messageFooter;

        talkerPub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}