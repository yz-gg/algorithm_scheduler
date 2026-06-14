#include <scheduler/mavros_command_interface.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
std::uint16_t slewPwm(std::uint16_t current, std::uint16_t target, int max_step)
{
    const int current_value = static_cast<int>(current);
    const int target_value = static_cast<int>(target);
    if (max_step <= 0 || std::abs(target_value - current_value) <= max_step)
    {
        return target;
    }

    return static_cast<std::uint16_t>(
        current_value + (target_value > current_value ? max_step : -max_step));
}

ros::Time messageStampOrNow(const ros::Time& stamp)
{
    return stamp.isZero() ? ros::Time::now() : stamp;
}

bool quaternionToEuler(
    double x,
    double y,
    double z,
    double w,
    double* roll,
    double* pitch,
    double* yaw)
{
    if (roll == nullptr || pitch == nullptr || yaw == nullptr ||
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(w))
    {
        return false;
    }

    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (norm < 1e-9)
    {
        return false;
    }

    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;

    const double sin_roll = 2.0 * (w * x + y * z);
    const double cos_roll = 1.0 - 2.0 * (x * x + y * y);
    *roll = std::atan2(sin_roll, cos_roll);

    const double sin_pitch = 2.0 * (w * y - z * x);
    if (std::fabs(sin_pitch) >= 1.0)
    {
        *pitch = std::copysign(1.5707963267948966, sin_pitch);
    }
    else
    {
        *pitch = std::asin(sin_pitch);
    }

    const double sin_yaw = 2.0 * (w * z + x * y);
    const double cos_yaw = 1.0 - 2.0 * (y * y + z * z);
    *yaw = std::atan2(sin_yaw, cos_yaw);
    return true;
}
}

MavrosCommandInterface::MavrosCommandInterface(
    ros::NodeHandle nh,
    ros::NodeHandle pnh)
    : nh_(std::move(nh))
    , pnh_(std::move(pnh))
{
    RcCommandMapperConfig rc_config;
    pnh_.param("rc_center_pwm", rc_config.center_pwm, rc_config.center_pwm);
    pnh_.param("rc_slow_max_pwm", rc_config.slow_max_pwm, rc_config.slow_max_pwm);
    pnh_.param(
        "rc_medium_max_pwm", rc_config.medium_max_pwm, rc_config.medium_max_pwm);
    pnh_.param("rc_fast_max_pwm", rc_config.fast_max_pwm, rc_config.fast_max_pwm);
    pnh_.param(
        "rc_surge_direction", rc_config.surge_direction, rc_config.surge_direction);
    pnh_.param(
        "rc_sway_direction", rc_config.sway_direction, rc_config.sway_direction);
    pnh_.param(
        "rc_heave_direction", rc_config.heave_direction, rc_config.heave_direction);
    pnh_.param("rc_yaw_direction", rc_config.yaw_direction, rc_config.yaw_direction);
    pnh_.param(
        "rc_input_deadband", rc_config.input_deadband, rc_config.input_deadband);
    pnh_.param("rc_pwm_slew_step", rc_pwm_slew_step_, rc_pwm_slew_step_);

    rc_mapper_ = RcCommandMapper(rc_config);
    last_rc_pwm_ = rc_mapper_.neutral();

    std::string state_topic = "/mavros/state";
    std::string imu_topic = "/mavros/imu/data";
    std::string local_pose_topic = "/mavros/local_position/pose";
    std::string rc_override_topic = "/mavros/rc/override";
    pnh_.param<std::string>("state_topic", state_topic, state_topic);
    pnh_.param<std::string>("imu_topic", imu_topic, imu_topic);
    pnh_.param<std::string>(
        "local_pose_topic", local_pose_topic, local_pose_topic);
    pnh_.param<std::string>(
        "rc_override_topic", rc_override_topic, rc_override_topic);

    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        state_topic, 10, &MavrosCommandInterface::stateCb, this);
    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>(
        imu_topic, 20, &MavrosCommandInterface::imuCb, this);
    local_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        local_pose_topic, 20, &MavrosCommandInterface::localPoseCb, this);

    vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(
        "/mavros/setpoint_velocity/cmd_vel", 10);

    local_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);
    rc_override_pub_ = nh_.advertise<mavros_msgs::OverrideRCIn>(
        rc_override_topic, 10);

    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");

    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");
}

void MavrosCommandInterface::stateCb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state_ = *msg;
    feedback_.state_valid = true;
    feedback_.connected = msg->connected;
    feedback_.armed = msg->armed;
    feedback_.mode = msg->mode;
    feedback_.state_stamp = ros::Time::now();
}

void MavrosCommandInterface::imuCb(const sensor_msgs::Imu::ConstPtr& msg)
{
    const geometry_msgs::Vector3& rate = msg->angular_velocity;
    const geometry_msgs::Vector3& acceleration = msg->linear_acceleration;
    feedback_.angular_velocity_valid = std::isfinite(rate.x) &&
        std::isfinite(rate.y) && std::isfinite(rate.z);
    feedback_.linear_acceleration_valid = std::isfinite(acceleration.x) &&
        std::isfinite(acceleration.y) && std::isfinite(acceleration.z);

    const ros::Time stamp = messageStampOrNow(msg->header.stamp);

    if (feedback_.angular_velocity_valid)
    {
        feedback_.roll_rate = rate.x;
        feedback_.pitch_rate = rate.y;
        feedback_.yaw_rate = rate.z;
        feedback_.angular_velocity_stamp = stamp;
    }

    if (feedback_.linear_acceleration_valid)
    {
        feedback_.body_ax = acceleration.x;
        feedback_.body_ay = acceleration.y;
        feedback_.body_az = acceleration.z;
        feedback_.linear_acceleration_stamp = stamp;
    }
}

void MavrosCommandInterface::localPoseCb(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    const geometry_msgs::Point& position = msg->pose.position;
    feedback_.local_position_valid = std::isfinite(position.x) &&
        std::isfinite(position.y) && std::isfinite(position.z);

    if (feedback_.local_position_valid)
    {
        feedback_.local_x = position.x;
        feedback_.local_y = position.y;
        feedback_.local_z = position.z;
    }

    const geometry_msgs::Quaternion& orientation = msg->pose.orientation;
    feedback_.attitude_valid = quaternionToEuler(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        &feedback_.roll,
        &feedback_.pitch,
        &feedback_.yaw);

    feedback_.local_pose_stamp = messageStampOrNow(msg->header.stamp);
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

bool MavrosCommandInterface::setMode(const std::string& mode_name)
{
    mavros_msgs::SetMode srv;
    srv.request.base_mode = 0;
    srv.request.custom_mode = mode_name;

    if (set_mode_client_.call(srv))
    {
        return srv.response.mode_sent;
    }

    return false;
}

bool MavrosCommandInterface::setCustomMode(uint32_t custom_mode_id)
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

bool MavrosCommandInterface::isArmed() const
{
    return current_state_.armed;
}

bool MavrosCommandInterface::getFeedback(Feedback* feedback) const
{
    if (feedback == nullptr)
    {
        return false;
    }

    *feedback = feedback_;
    if (feedback->angular_velocity_valid)
    {
        feedback->stamp = feedback->angular_velocity_stamp;
    }
    else if (feedback->linear_acceleration_valid)
    {
        feedback->stamp = feedback->linear_acceleration_stamp;
    }
    else if (feedback->attitude_valid || feedback->local_position_valid)
    {
        feedback->stamp = feedback->local_pose_stamp;
    }
    else if (feedback->state_valid)
    {
        feedback->stamp = feedback->state_stamp;
    }

    return feedback->state_valid || feedback->angular_velocity_valid ||
        feedback->linear_acceleration_valid || feedback->attitude_valid ||
        feedback->local_position_valid;
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
    case CommandType::RC_OVERRIDE:
        publishRcOverride(command.rc_override);
        break;
    case CommandType::RC_RELEASE:
        releaseRcOverride();
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

void MavrosCommandInterface::publishRcOverride(
    const RcNormalizedCommand& command)
{
    const RcPwmCommand target = rc_mapper_.map(command);
    const RcPwmCommand neutral_pwm = rc_mapper_.neutral();
    const bool neutral = target.surge == neutral_pwm.surge &&
        target.sway == neutral_pwm.sway &&
        target.heave == neutral_pwm.heave &&
        target.yaw == neutral_pwm.yaw;

    last_rc_pwm_ = neutral ? neutral_pwm : applyRcSlewLimit(target);

    ROS_INFO_STREAM_THROTTLE(
        1.0,
        "RC override gear=" << RcSpeedGearToString(command.gear)
                            << " surge=" << last_rc_pwm_.surge
                            << " sway=" << last_rc_pwm_.sway
                            << " heave=" << last_rc_pwm_.heave
                            << " yaw=" << last_rc_pwm_.yaw);

    transmitRcPwm(last_rc_pwm_);
}

void MavrosCommandInterface::releaseRcOverride()
{
    mavros_msgs::OverrideRCIn msg;
    const RcOverrideChannels channels = BuildRcReleaseChannels();
    if (msg.channels.size() != channels.size())
    {
        ROS_ERROR_THROTTLE(
            1.0,
            "MAVROS OverrideRCIn channel count mismatch: expected %zu, got %zu",
            channels.size(),
            msg.channels.size());
        return;
    }

    std::copy(channels.begin(), channels.end(), msg.channels.begin());
    rc_override_pub_.publish(msg);
    last_rc_pwm_ = rc_mapper_.neutral();
    ROS_INFO_THROTTLE(1.0, "Released all RC override channels");
}

RcPwmCommand MavrosCommandInterface::applyRcSlewLimit(
    const RcPwmCommand& target) const
{
    RcPwmCommand limited;
    limited.surge = slewPwm(last_rc_pwm_.surge, target.surge, rc_pwm_slew_step_);
    limited.sway = slewPwm(last_rc_pwm_.sway, target.sway, rc_pwm_slew_step_);
    limited.heave = slewPwm(last_rc_pwm_.heave, target.heave, rc_pwm_slew_step_);
    limited.yaw = slewPwm(last_rc_pwm_.yaw, target.yaw, rc_pwm_slew_step_);
    return limited;
}

bool MavrosCommandInterface::transmitRcPwm(const RcPwmCommand& command)
{
    mavros_msgs::OverrideRCIn msg;
    const std::uint16_t center = rc_mapper_.neutral().surge;
    const RcOverrideChannels channels =
        BuildRcOverrideChannels(command, center);
    if (msg.channels.size() != channels.size())
    {
        ROS_ERROR_THROTTLE(
            1.0,
            "MAVROS OverrideRCIn channel count mismatch: expected %zu, got %zu",
            channels.size(),
            msg.channels.size());
        return false;
    }

    std::copy(channels.begin(), channels.end(), msg.channels.begin());

    rc_override_pub_.publish(msg);
    return true;
}
