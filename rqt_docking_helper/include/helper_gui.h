#ifndef rqt_docking_helper__HELPER_GUI_H
#define rqt_docking_helper__HELPER_GUI_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_helper_gui.h>
#include <QWidget>
#include <QFlags>
#include <ros/ros.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

#include "april_docking/DockingAction.h"


namespace rqt_docking_helper {

class helper_gui : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    helper_gui();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
    Ui::HelperWidget ui_;
    QWidget* widget_;

    void toggleActive();
    void toggleInactive();

    void callPauseService();
    void callUnpauseService();
    void callSimSpeedService();

protected:
    ros::Publisher docking_goal_pub_;
    ros::Publisher amcl_pose_pub_;
    ros::ServiceClient simSpeedClient;
    ros::ServiceClient pauseClient;
    ros::ServiceClient unpauseClient;
    gazebo_msgs::SetPhysicsProperties phyProps;
    geometry_msgs::PoseWithCovarianceStamped amclPose;
    std_srvs::Empty dummyRequest;
    april_docking::DockingActionGoal output_;

private slots:
    void on_pause_sim_button_toggled(bool checked);
    void on_speed_05x_button_clicked();
    void on_speed_1x_button_clicked();
    void on_speed_2x_button_clicked();
    void on_speed_3x_button_clicked();
    void on_setpos_spawn_button_clicked();
    void on_setpos_0_button_clicked();
    void on_setpos_4_button_clicked();
    void on_undock_button_clicked();
    void on_dock0_button_clicked();
    void on_dock4_button_clicked();
};

} //namespace
#endif // rqt_docking_helper__HELPER_GUI_H
