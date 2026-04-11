#include <scheduler/simple_mission_controller.h>

#include <algorithm>
#include <utility>

#include <scheduler/ControlModule.h>

namespace
{
double clampLower(double value, double lower_bound)
{
    return std::max(value, lower_bound);
}
}

SimpleMissionController::SimpleMissionController(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    VehicleCommandInterface* vehicle,
    std::string mission_type,
    std::string default_module_name)
    : nh_(std::move(nh))
    , pnh_(std::move(pnh))
    , vehicle_(vehicle)
    , mission_type_(std::move(mission_type))
{
    loadParameters(default_module_name);
    module_control_client_ = nh_.serviceClient<scheduler::ControlModule>(
        "/launch_manager_node/control_module");
}

void SimpleMissionController::run()
{
    ros::Rate rate(clampLower(config_.command_rate_hz, 1.0));

    ROS_INFO_STREAM(
        "Simple mission controller started for mission type ["
        << mission_type_ << "]");

    while (ros::ok())
    {
        ros::spinOnce();
        maybeStartModule();
        vehicle_->publishCommand(buildCommand());
        rate.sleep();
    }

    maybeStopModule();
    vehicle_->publishHold();
}

std::string SimpleMissionController::missionType() const
{
    return mission_type_;
}

void SimpleMissionController::loadParameters(const std::string& default_module_name)
{
    config_.autostart_module = default_module_name;

    pnh_.param<std::string>(
        "autostart_module", config_.autostart_module, config_.autostart_module);
    pnh_.param(
        "auto_start_module", config_.auto_start_module, config_.auto_start_module);
    pnh_.param(
        "command_rate_hz", config_.command_rate_hz, config_.command_rate_hz);
    pnh_.param(
        "module_request_interval_sec",
        config_.module_request_interval_sec,
        config_.module_request_interval_sec);
    pnh_.param(
        "service_wait_sec", config_.service_wait_sec, config_.service_wait_sec);

    pnh_.param<std::string>(
        "command_type", config_.command_type, config_.command_type);

    pnh_.param("body_vx", config_.body_vx, config_.body_vx);
    pnh_.param("body_vy", config_.body_vy, config_.body_vy);
    pnh_.param("body_vz", config_.body_vz, config_.body_vz);
    pnh_.param(
        "body_yaw_rate", config_.body_yaw_rate, config_.body_yaw_rate);

    pnh_.param("local_pose_x", config_.local_pose_x, config_.local_pose_x);
    pnh_.param("local_pose_y", config_.local_pose_y, config_.local_pose_y);
    pnh_.param("local_pose_z", config_.local_pose_z, config_.local_pose_z);
    pnh_.param(
        "local_pose_yaw", config_.local_pose_yaw, config_.local_pose_yaw);
}

bool SimpleMissionController::tryCallModuleCommand(
    const std::string& module_name,
    const std::string& command)
{
    if (!module_control_client_.waitForExistence(ros::Duration(config_.service_wait_sec)))
    {
        ROS_WARN_THROTTLE(5.0, "Waiting for launch manager service");
        return false;
    }

    scheduler::ControlModule srv;
    srv.request.module_name = module_name;
    srv.request.command = command;

    if (!module_control_client_.call(srv))
    {
        ROS_WARN_THROTTLE(5.0, "Failed to call launch manager service");
        return false;
    }

    if (!srv.response.success)
    {
        ROS_WARN_THROTTLE(
            5.0,
            "Module command [%s] for [%s] failed: %s",
            command.c_str(),
            module_name.c_str(),
            srv.response.message.c_str());
        return false;
    }

    return true;
}

bool SimpleMissionController::maybeStartModule()
{
    if (!config_.auto_start_module || config_.autostart_module.empty() || module_started_)
    {
        return true;
    }

    const ros::Time now = ros::Time::now();
    if (!last_module_request_time_.isZero() &&
        (now - last_module_request_time_).toSec() < config_.module_request_interval_sec)
    {
        return false;
    }

    last_module_request_time_ = now;
    module_started_ = tryCallModuleCommand(config_.autostart_module, "start");
    return module_started_;
}

void SimpleMissionController::maybeStopModule()
{
    if (!module_started_ || config_.autostart_module.empty())
    {
        return;
    }

    if (tryCallModuleCommand(config_.autostart_module, "stop"))
    {
        module_started_ = false;
    }
}

VehicleCommandInterface::Command SimpleMissionController::buildCommand() const
{
    if (config_.command_type == "body_velocity")
    {
        return VehicleCommandInterface::BodyVelocity(
            config_.body_vx,
            config_.body_vy,
            config_.body_vz,
            config_.body_yaw_rate);
    }

    if (config_.command_type == "local_pose")
    {
        return VehicleCommandInterface::LocalPose(
            config_.local_pose_x,
            config_.local_pose_y,
            config_.local_pose_z,
            config_.local_pose_yaw);
    }

    return VehicleCommandInterface::Hold();
}
