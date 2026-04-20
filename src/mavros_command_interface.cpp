#include <scheduler/mavros_command_interface.h>

#include <cmath>
#include <utility>

MavrosCommandInterface::MavrosCommandInterface(ros::NodeHandle nh)
    : nh_(std::move(nh))
{
    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, &MavrosCommandInterface::stateCb, this);

    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);

    local_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");
}

void MavrosCommandInterface::stateCb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
}

bool MavrosCommandInterface::waitForConnection()
{
    ros::Rate rate(10.0);
    while (ros::ok() && !current_state_.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    return current_state_.connected;
}

bool MavrosCommandInterface::waitForServices(const ros::Duration& timeout)
{
    bool services_ready = true;

    if (!set_mode_client_.waitForExistence(timeout))
    {
        ROS_WARN("Set mode service is not available yet");
        services_ready = false;
    }

    if (!arming_client_.waitForExistence(timeout))
    {
        ROS_WARN("Arming service is not available yet");
        services_ready = false;
    }

    return services_ready;
}

bool MavrosCommandInterface::setMode(uint32_t custom_mode_id)
{
    mavros_msgs::SetMode srv;
    srv.request.base_mode = 0;
    srv.request.custom_mode = std::to_string(custom_mode_id);

    ROS_INFO("Request mode switch: base_mode=%u, custom_mode=%s",
             srv.request.base_mode,
             srv.request.custom_mode.c_str());

    if (set_mode_client_.call(srv))
    {
        return srv.response.mode_sent;
    }

    ROS_INFO("SetMode response: mode_sent=%s",
             srv.response.mode_sent ? "true" : "false");

    return false;
}

bool MavrosCommandInterface::arm(bool value)
{
    mavros_msgs::CommandBool srv;
    srv.request.value = value;

    if (arming_client_.call(srv))
    {
        return srv.response.success;
    }

    return false;
}

void MavrosCommandInterface::publishCommand(const Command& command)
{
    switch (command.type)
    {
    case CommandType::BODY_VELOCITY:
        publishBodyVelocity(command.body_velocity);
        break;
    case CommandType::LOCAL_POSE:
        publishLocalPose(command.local_pose);
        break;
    case CommandType::HOLD:
    default:
        publishHold();
        break;
    }
}

void MavrosCommandInterface::publishHold()
{
    publishBodyVelocity(BodyVelocityCommand{});
}

void MavrosCommandInterface::publishBodyVelocity(
    const BodyVelocityCommand& command)
{
    geometry_msgs::TwistStamped cmd;
    cmd.header.stamp = ros::Time::now();

    cmd.twist.linear.x = command.vx;
    cmd.twist.linear.y = command.vy;
    cmd.twist.linear.z = command.vz;
    cmd.twist.angular.x = command.roll_rate;
    cmd.twist.angular.y = command.pitch_rate;
    cmd.twist.angular.z = command.yaw_rate;

    vel_pub_.publish(cmd);
}

void MavrosCommandInterface::publishLocalPose(
    const LocalPoseCommand& command)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = command.x;
    pose.pose.position.y = command.y;
    pose.pose.position.z = command.z;
    pose.pose.orientation.z = std::sin(0.5 * command.yaw);
    pose.pose.orientation.w = std::cos(0.5 * command.yaw);

    local_pose_pub_.publish(pose);
}
