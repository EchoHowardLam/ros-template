#include "helper_gui.h"
#include <pluginlib/class_list_macros.h>

namespace rqt_docking_helper {

helper_gui::helper_gui()
    : rqt_gui_cpp::Plugin(), widget_(0)
{
    setObjectName("Helper GUI");
    boost::array<double, 36UL> dummyCov = {
        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
    };
    phyProps.request.time_step = 0.001;
    phyProps.request.max_update_rate = 1000.0;
    phyProps.request.gravity.x = 0.0;
    phyProps.request.gravity.y = 0.0;
    phyProps.request.gravity.z = -9.8;
    phyProps.request.ode_config.auto_disable_bodies = false,
    phyProps.request.ode_config.sor_pgs_precon_iters = 150,
    phyProps.request.ode_config.sor_pgs_iters = 150,
    phyProps.request.ode_config.sor_pgs_w = 1.4,
    phyProps.request.ode_config.sor_pgs_rms_error_tol = 0.0,
    phyProps.request.ode_config.contact_surface_layer = 0.01,
    phyProps.request.ode_config.contact_max_correcting_vel = 2000.0,
    phyProps.request.ode_config.cfm = 0.00001,
    phyProps.request.ode_config.erp = 0.2,
    phyProps.request.ode_config.max_contacts = 10;
    amclPose.header.frame_id = "map";
    amclPose.pose.pose.position.x = 0.0;
    amclPose.pose.pose.position.y = 0.0;
    amclPose.pose.pose.position.z = 0.0;
    amclPose.pose.pose.orientation.x = 0.0;
    amclPose.pose.pose.orientation.y = 0.0;
    amclPose.pose.pose.orientation.z = 0.0;
    amclPose.pose.pose.orientation.w = 0.0;
    amclPose.pose.covariance = dummyCov;
}

void helper_gui::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // create QWidget
    widget_ = new QWidget();

    // access standalone command line arguments
    QStringList argv = context.argv();

    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    //Publisher
    pauseClient = getNodeHandle().serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    unpauseClient = getNodeHandle().serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    simSpeedClient = getNodeHandle().serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties", 10);
    docking_goal_pub_ = getNodeHandle().advertise<april_docking::DockingActionGoal>("/docking/goal", 10);
    amcl_pose_pub_ = getNodeHandle().advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);

    // Set EStop to active (checked)
    ui_.pause_sim_button->setCheckable(true);
    ui_.pause_sim_button->setChecked(false);
    toggleInactive();

    // CONNECTIONS
    connect(ui_.pause_sim_button, SIGNAL( toggled(bool) ), this, SLOT( on_pause_sim_button_toggled(bool) ) );
    connect(ui_.speed_05x_button, SIGNAL( clicked() ), this, SLOT( on_speed_05x_button_clicked() ) );
    connect(ui_.speed_1x_button, SIGNAL( clicked() ), this, SLOT( on_speed_1x_button_clicked() ) );
    connect(ui_.speed_2x_button, SIGNAL( clicked() ), this, SLOT( on_speed_2x_button_clicked() ) );
    connect(ui_.speed_3x_button, SIGNAL( clicked() ), this, SLOT( on_speed_3x_button_clicked() ) );
    connect(ui_.setpos_spawn_button, SIGNAL( clicked() ), this, SLOT( on_setpos_spawn_button_clicked() ) );
    connect(ui_.setpos_0_button, SIGNAL( clicked() ), this, SLOT( on_setpos_0_button_clicked() ) );
    connect(ui_.setpos_4_button, SIGNAL( clicked() ), this, SLOT( on_setpos_4_button_clicked() ) );
    connect(ui_.undock_button, SIGNAL( clicked() ), this, SLOT( on_undock_button_clicked() ) );
    connect(ui_.dock0_button, SIGNAL( clicked() ), this, SLOT( on_dock0_button_clicked() ) );
    connect(ui_.dock4_button, SIGNAL( clicked() ), this, SLOT( on_dock4_button_clicked() ) );
}

void helper_gui::shutdownPlugin()
{
}

void helper_gui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void helper_gui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void helper_gui::toggleActive()
{
    callUnpauseService();
    ui_.pause_sim_button->setStyleSheet("background-color: red; color: white; font: 20pt \"Ubuntu\";");
    ui_.pause_sim_button->setText("┃┃");
}

void helper_gui::toggleInactive()
{
    callPauseService();
    ui_.pause_sim_button->setStyleSheet("background-color: rgb(0,255,0); color: black; font: 15pt \"Ubuntu\";");
    ui_.pause_sim_button->setText("▶");
}

void helper_gui::callPauseService() {
    if (pauseClient.call(dummyRequest))
        ROS_INFO("Paused");
    else
        ROS_ERROR("Failed to call service /gazebo/pause_physics");
}
void helper_gui::callUnpauseService() {
    if (unpauseClient.call(dummyRequest))
        ROS_INFO("Unpaused");
    else
        ROS_ERROR("Failed to call service /gazebo/unpause_physics");
}
void helper_gui::callSimSpeedService() {
    if (simSpeedClient.call(phyProps))
        ROS_INFO("Changed sim speed to 0.1x");
    else
        ROS_ERROR("Failed to call service /gazebo/set_physics_properties");
}

void rqt_docking_helper::helper_gui::on_pause_sim_button_toggled(bool checked)
{
    if (checked)
        toggleActive();
    else
        toggleInactive();
}

void rqt_docking_helper::helper_gui::on_speed_05x_button_clicked()
{
    phyProps.request.max_update_rate = 500.0;
    callSimSpeedService();
}
void rqt_docking_helper::helper_gui::on_speed_1x_button_clicked()
{
    phyProps.request.max_update_rate = 1000.0;
    callSimSpeedService();
}
void rqt_docking_helper::helper_gui::on_speed_2x_button_clicked()
{
    phyProps.request.max_update_rate = 2000.0;
    callSimSpeedService();
}
void rqt_docking_helper::helper_gui::on_speed_3x_button_clicked()
{
    phyProps.request.max_update_rate = 3000.0;
    callSimSpeedService();
}
void rqt_docking_helper::helper_gui::on_setpos_spawn_button_clicked()
{
    amclPose.pose.pose.position.x = 4.88801860809;
    amclPose.pose.pose.position.y = 2.12388205528;
    amclPose.pose.pose.orientation.z = 0.00157561065254;
    amclPose.pose.pose.orientation.w = 0.999998758725;
    amcl_pose_pub_.publish(amclPose);
}
void rqt_docking_helper::helper_gui::on_setpos_0_button_clicked()
{
    amclPose.pose.pose.position.x = 4.56910943985;
    amclPose.pose.pose.position.y = 1.2646420002;
    amclPose.pose.pose.orientation.z = -0.999993710863;
    amclPose.pose.pose.orientation.w = 0.00354658065717;
    amcl_pose_pub_.publish(amclPose);
}
void rqt_docking_helper::helper_gui::on_setpos_4_button_clicked()
{
    amclPose.pose.pose.position.x = 4.57685279846;
    amclPose.pose.pose.position.y = 2.72769403458;
    amclPose.pose.pose.orientation.z = 0.999991558486;
    amclPose.pose.pose.orientation.w = 0.00410888755734;
    amcl_pose_pub_.publish(amclPose);
}

void rqt_docking_helper::helper_gui::on_undock_button_clicked()
{
    output_.goal.goalId = "-1";
    ui_.status_text->setText("Undocking");
    docking_goal_pub_.publish(output_);
}

void rqt_docking_helper::helper_gui::on_dock0_button_clicked()
{
    output_.goal.goalId = "0";
    ui_.status_text->setText("Docking 0");
    docking_goal_pub_.publish(output_);
}

void rqt_docking_helper::helper_gui::on_dock4_button_clicked()
{
    output_.goal.goalId = "4";
    ui_.status_text->setText("Docking 4");
    docking_goal_pub_.publish(output_);
}


}  // namespace
PLUGINLIB_EXPORT_CLASS(rqt_docking_helper::helper_gui, rqt_gui_cpp::Plugin)
