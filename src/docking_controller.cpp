#include <scheduler/docking_controller.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
double clampValue(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}
}

DockingController::DockingController(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    VehicleCommandInterface* vehicle)
    : nh_(std::move(nh))
    , pnh_(std::move(pnh))
    , vehicle_(vehicle)
{
    loadParameters();

    module_control_client_ = nh_.serviceClient<scheduler::ControlModule>(
        "/launch_manager_node/control_module");
}

void DockingController::run()
{
    ros::Rate rate(std::max(config_.command_rate_hz, 1.0));

    ROS_INFO("Docking controller started");

    if (config_.auto_start_inspection)
    {
        enterState(State::START, "docking state machine booted");
    }
    else
    {
        enterState(
            State::SEARCH_BLUE_LIGHT,
            "auto start disabled, waiting for blue light detections");
    }

    while (ros::ok())
    {
        ros::spinOnce();
        step();
        checkObversationFresh();
        rate.sleep();
    }

    stopModuleIfNeeded();
    vehicle_->publishHold();
}

std::string DockingController::missionType() const
{
    return "docking";
}

void DockingController::loadParameters()
{
    pnh_.param<std::string>(
        "autostart_module", config_.autostart_module, config_.autostart_module);
    pnh_.param<std::string>(
        "topics/blue_light", config_.blue_light_topic, config_.blue_light_topic);
    pnh_.param<std::string>(
        "topics/dock_pose", config_.dock_pose_topic, config_.dock_pose_topic);
    pnh_.param<std::string>(
        "topics/apriltag", config_.apriltag_topic, config_.apriltag_topic);

    pnh_.param(
        "auto_start_inspection",
        config_.auto_start_inspection,
        config_.auto_start_inspection);
    pnh_.param("command_rate_hz", config_.command_rate_hz, config_.command_rate_hz);
    pnh_.param(
        "observation_timeout_sec",
        config_.observation_timeout_sec,
        config_.observation_timeout_sec);
    pnh_.param(
        "module_request_interval_sec",
        config_.module_request_interval_sec,
        config_.module_request_interval_sec);
    pnh_.param(
        "service_wait_sec", config_.service_wait_sec, config_.service_wait_sec);

    pnh_.param(
        "search/scan_yaw_rate",
        config_.search.scan_yaw_rate,
        config_.search.scan_yaw_rate);
    pnh_.param(
        "search/timeout_sec",
        config_.search.timeout_sec,
        config_.search.timeout_sec);
    pnh_.param(
        "search/retry_times",
        config_.search.retry_times,
        config_.search.retry_times);
    pnh_.param(
        "search/stable_frames",
        config_.search.stable_frames,
        config_.search.stable_frames);
    loadMotionLimits("search", &config_.search.command_limits);

    pnh_.param(
        "approach/forward_speed",
        config_.approach.forward_speed,
        config_.approach.forward_speed);
    pnh_.param(
        "approach/yaw_kp",
        config_.approach.yaw_kp,
        config_.approach.yaw_kp);
    pnh_.param(
        "approach/heave_kp",
        config_.approach.heave_kp,
        config_.approach.heave_kp);
    pnh_.param(
        "approach/stable_frames",
        config_.approach.stable_frames,
        config_.approach.stable_frames);
    loadMotionLimits("approach", &config_.approach.command_limits);

    pnh_.param(
        "align/yaw_deg_tolerance",
        config_.align.yaw_deg_tolerance,
        config_.align.yaw_deg_tolerance);
    pnh_.param(
        "align/depth_m_tolerance",
        config_.align.depth_m_tolerance,
        config_.align.depth_m_tolerance);
    pnh_.param(
        "align/sway_m_tolerance",
        config_.align.sway_m_tolerance,
        config_.align.sway_m_tolerance);
    pnh_.param(
        "align/dist_tolerance",
        config_.align.dist_tolerance,
        config_.align.dist_tolerance);
    pnh_.param("align/yaw_kp", config_.align.yaw_kp, config_.align.yaw_kp);
    pnh_.param("align/sway_kp", config_.align.sway_kp, config_.align.sway_kp);
    pnh_.param("align/heave_kp", config_.align.heave_kp, config_.align.heave_kp);
    pnh_.param(
        "align/surge_speed",
        config_.align.surge_speed,
        config_.align.surge_speed);
    pnh_.param(
        "align/stable_frames",
        config_.align.stable_frames,
        config_.align.stable_frames);
    loadMotionLimits("align", &config_.align.command_limits);

    pnh_.param(
        "align_with_tag/yaw_deg_tolerance",
        config_.align_with_tag.yaw_deg_tolerance,
        config_.align_with_tag.yaw_deg_tolerance);
    pnh_.param(
        "align_with_tag/depth_m_tolerance",
        config_.align_with_tag.depth_m_tolerance,
        config_.align_with_tag.depth_m_tolerance);
    pnh_.param(
        "align_with_tag/sway_m_tolerance",
        config_.align_with_tag.sway_m_tolerance,
        config_.align_with_tag.sway_m_tolerance);
    pnh_.param(
        "align_with_tag/yaw_kp",
        config_.align_with_tag.yaw_kp,
        config_.align_with_tag.yaw_kp);
    pnh_.param(
        "align_with_tag/sway_kp",
        config_.align_with_tag.sway_kp,
        config_.align_with_tag.sway_kp);
    pnh_.param(
        "align_with_tag/heave_kp",
        config_.align_with_tag.heave_kp,
        config_.align_with_tag.heave_kp);
    pnh_.param(
        "align_with_tag/stable_frames",
        config_.align_with_tag.stable_frames,
        config_.align_with_tag.stable_frames);
    loadMotionLimits("align_with_tag", &config_.align_with_tag.command_limits);

    pnh_.param(
        "enter_dock/forward_speed",
        config_.enter_dock.forward_speed,
        config_.enter_dock.forward_speed);
    pnh_.param(
        "enter_dock/max_duration_sec",
        config_.enter_dock.max_duration_sec,
        config_.enter_dock.max_duration_sec);
    loadMotionLimits("enter_dock", &config_.enter_dock.command_limits);

    pnh_.param(
        "capture/timeout_sec",
        config_.capture.timeout_sec,
        config_.capture.timeout_sec);
}

void DockingController::loadMotionLimits(
    const std::string& prefix,
    MotionLimitConfig* limits)
{
    if (limits == nullptr)
    {
        return;
    }

    pnh_.param(prefix + "/max_surge", limits->surge, limits->surge);
    pnh_.param(prefix + "/max_sway", limits->sway, limits->sway);
    pnh_.param(prefix + "/max_heave", limits->heave, limits->heave);
    pnh_.param(prefix + "/max_yaw", limits->yaw, limits->yaw);
}

void DockingController::step()
{
    switch (state_)
    {
    case State::START:
        handleStart();
        break;
    case State::SEARCH_BLUE_LIGHT:
        handleSearch();
        break;
    case State::APPROACH_BLUE_LIGHT:
        handleApproach();
        break;
    case State::ALIGN_WITH_DOCK:
        handleAlign();
        break;
    case State::ALIGN_WITH_APRILTAG:
        handleAlignWithTag();
        break;
    case State::ENTER_DOCK:
        handleEnterDock();
        break;
    case State::WAIT_CAPTURE:
        handleCaptured();
        break;
    case State::COMPLETED:
        handleCompleted();
        break;
    case State::ERROR:
        handleError();
        break;
    }
}

void DockingController::enterState(
    State next_state,
    const std::string& reason)
{
    if (state_ == next_state && !state_entered_at_.isZero())
    {
        return;
    }

    ROS_INFO_STREAM(
        "Docking state transition [" << stateToString(state_) << "] -> ["
        << stateToString(next_state) << "]"
        << (reason.empty() ? "" : ": " + reason));

    state_ = next_state;
    state_entered_at_ = ros::Time::now();
}

std::string DockingController::stateToString(State state) const
{
    switch (state)
    {
    case State::START:
        return "START";
    case State::SEARCH_BLUE_LIGHT:
        return "SEARCH_BLUE_LIGHT";
    case State::APPROACH_BLUE_LIGHT:
        return "APPROACH_BLUE_LIGHT";
    case State::ALIGN_WITH_DOCK:
        return "ALIGN_WITH_DOCK";
    case State::ALIGN_WITH_APRILTAG:
        return "ALIGN_WITH_APRILTAG";
    case State::ENTER_DOCK:
        return "ENTER_DOCK";
    case State::WAIT_CAPTURE:
        return "WAIT_CAPTURE";
    case State::COMPLETED:
        return "COMPLETED";
    case State::ERROR:
        return "ERROR";
    }

    return "UNKNOWN";
}

double DockingController::stateElapsedSec() const
{
    if (state_entered_at_.isZero())
    {
        return 0.0;
    }

    return (ros::Time::now() - state_entered_at_).toSec();
}

void DockingController::handleStart()
{
    vehicle_->publishHold();

    if (config_.autostart_module.empty())
    {
        enterState(State::SEARCH_BLUE_LIGHT, "no autostart module configured");
        return;
    }

    if (module_started_)
    {
        enterState(State::SEARCH_BLUE_LIGHT, "docking module already running");
        return;
    }

    const ros::Time now = ros::Time::now();
    if (!last_module_request_time_.isZero() &&
        (now - last_module_request_time_).toSec() < config_.module_request_interval_sec)
    {
        return;
    }

    last_module_request_time_ = now;

    scheduler::ControlModuleResponse response;
    if (!tryCallModuleCommand(config_.autostart_module, "start", &response))
    {
        return;
    }

    if (response.success)
    {
        module_started_ =
            (response.state == "RUNNING" || response.message == "module already running");
        enterState(State::SEARCH_BLUE_LIGHT, "docking module started");
        return;
    }

    ROS_WARN_THROTTLE(
        5.0,
        "Docking module [%s] failed to start: %s",
        config_.autostart_module.c_str(),
        response.message.c_str());
}

void DockingController::handleSearch()
{
    data_.clearCommand();
    data_.yaw = config_.search.scan_yaw_rate;
    vehicle_->publishCommand(buildCommand(state_));

    if (config_.search.timeout_sec > 0.0 &&
        stateElapsedSec() >= config_.search.timeout_sec)
    {
        ++search_retry_count_;
        if (search_retry_count_ > config_.search.retry_times)
        {
            enterState(State::ERROR, "blue light search timed out");
            return;
        }

        data_.blue_light_detected_count = 0;
        data_.blue_light_detected = false;
        data_.new_remote_light = false;
        state_entered_at_ = ros::Time::now();

        ROS_WARN_STREAM(
            "Retrying blue light search (" << search_retry_count_ << "/"
            << config_.search.retry_times << ")");
        return;
    }

    if (data_.blue_light_detected && data_.new_remote_light)
    {
        ++data_.blue_light_detected_count;
        data_.new_remote_light = false;

        if (data_.blue_light_detected_count >= config_.search.stable_frames)
        {
            search_retry_count_ = 0;
            data_.reset();
            enterState(State::APPROACH_BLUE_LIGHT, "blue light detection is stable");
        }

        return;
    }

    if (!data_.blue_light_detected)
    {
        data_.blue_light_detected_count = 0;
    }
}

void DockingController::handleApproach()
{
    if (data_.blue_light_detected && data_.new_remote_light)
    {
        data_.clearCommand();
        data_.yaw = config_.approach.yaw_kp * data_.blue_light_yaw_error;
        data_.surge = config_.approach.forward_speed;

        vehicle_->publishCommand(buildCommand(state_));
        data_.new_remote_light = false;
    }
    else
    {
        vehicle_->publishHold();
    }

    if (data_.dock_pose_valid && data_.new_dock_pose)
    {
        ++data_.dock_detected_count;
        data_.new_dock_pose = false;

        if (data_.dock_detected_count >= config_.approach.stable_frames)
        {
            data_.reset();
            enterState(State::ALIGN_WITH_DOCK, "dock pose detection is stable");
        }
    }
    else if (!data_.dock_pose_valid)
    {
        data_.dock_detected_count = 0;
    }
}

void DockingController::handleAlign()
{
    if (data_.dock_pose_valid && data_.new_dock_pose)
    {
        data_.clearCommand();
        data_.yaw = data_.dock_yaw_error * config_.align.yaw_kp;
        data_.sway = data_.dock_sway_error * config_.align.sway_kp;
        data_.heave = data_.dock_depth_error * config_.align.heave_kp;
        data_.surge =
            data_.dock_distance > config_.align.dist_tolerance ? config_.align.surge_speed
                                                               : 0.0;

        vehicle_->publishCommand(buildCommand(state_));
        data_.new_dock_pose = false;
    }
    else
    {
        vehicle_->publishHold();
    }

    const bool ready_for_tag = readyForTag();
    if (ready_for_tag && data_.apriltag_detected && data_.new_tag)
    {
        ++data_.tag_detected_count;
        data_.new_tag = false;

        if (data_.tag_detected_count >= config_.align.stable_frames)
        {
            data_.reset();
            enterState(
                State::ALIGN_WITH_APRILTAG,
                "apriltag detection is stable near dock alignment");
        }
    }
    else if (!ready_for_tag || !data_.apriltag_detected)
    {
        data_.tag_detected_count = 0;
    }
}

void DockingController::handleAlignWithTag()
{
    const bool has_new_tag = data_.apriltag_detected && data_.new_tag;
    if (has_new_tag)
    {
        data_.clearCommand();
        data_.sway = config_.align_with_tag.sway_kp * data_.tag_sway_error;
        data_.heave = config_.align_with_tag.heave_kp * data_.tag_depth_error;
        data_.yaw = config_.align_with_tag.yaw_kp * data_.tag_yaw_error;

        vehicle_->publishCommand(buildCommand(state_));
    }
    else
    {
        vehicle_->publishHold();
    }

    const bool ready_for_enter_dock = readyForEnterDock();
    if (has_new_tag && ready_for_enter_dock)
    {
        ++data_.tag_stable_count;

        if (data_.tag_stable_count >= config_.align_with_tag.stable_frames)
        {
            data_.new_tag = false;
            data_.reset();
            enterState(State::ENTER_DOCK, "apriltag alignment is stable");
            return;
        }
    }
    else if (!ready_for_enter_dock || !data_.apriltag_detected)
    {
        data_.tag_stable_count = 0;
    }

    data_.new_tag = false;
}

void DockingController::handleEnterDock()
{
    data_.clearCommand();
    data_.surge = config_.enter_dock.forward_speed;
    vehicle_->publishCommand(buildCommand(state_));

    if (config_.enter_dock.max_duration_sec > 0.0 &&
        stateElapsedSec() >= config_.enter_dock.max_duration_sec)
    {
        enterState(State::WAIT_CAPTURE, "enter dock duration reached");
    }
}

void DockingController::handleCaptured()
{
    vehicle_->publishHold();

    if (data_.captured)
    {
        enterState(State::COMPLETED, "capture feedback received");
        return;
    }

    if (config_.capture.timeout_sec > 0.0 &&
        stateElapsedSec() >= config_.capture.timeout_sec)
    {
        enterState(State::ERROR, "capture wait timed out");
    }
}

void DockingController::handleCompleted()
{
    vehicle_->publishHold();
    stopModuleIfNeeded();
}

void DockingController::handleError()
{
    vehicle_->publishHold();
    stopModuleIfNeeded();
}

void DockingController::RuntimeData::reset()
{
    *this = RuntimeData{};
}

/* todo
    topic
*/
void DockingController::BlueLightCB()
{
    data_.blue_light_detected = false;
    data.blue_light_yaw_error = 0;

    data_.new_remote_light = true;
    data_.light_timestamp = ros::Time::now();
}

void DockingController::DockPoseCB()
{
    data_.dock_pose_valid = false;
    data_.dock_yaw_error = 0;
    data_.dock_depth_error = 0;
    data_.dock_sway_error = 0;
    data_.dock_distance = 0;

    data_.new_dock_pose = true;
    data_.dock_pose_timestamp = ros::Time::now();
}

void DockingController::AprilTagCB()
{
    data_.apriltag_detected = false;
    data_.tag_sway_error = 0;
    data_.tag_yaw_error = 0;
    data_.tag_depth_error = 0;

    data_.new_tag = true;
    data_.apriltag_timestamp = ros::Time::now();
}

void DockingController::checkObversationFresh()
{
    if (ros::Time::now() - data_.light_timestamp > config_.approach.fresh_tolerance_s)
    {
        data_.new_remote_light = false;
    }

    if (ros::Time::now() - data_.dock_pose_timestamp > config_.align.fresh_tolerance_s)
    {
        data_.new_dock_pose = false;
    }

    if (ros::Time::now() - data_.apriltag_timestamp > config_.align_with_tag.fresh_tolerance_s)
    {
        data_.new_tag = false;
    }
}

void DockingController::RuntimeData::clearCommand()
{
    surge = 0.0;
    sway = 0.0;
    heave = 0.0;
    yaw = 0.0;
}

bool DockingController::readyForTag() const
{
    return std::abs(data_.dock_yaw_error) <= config_.align.yaw_deg_tolerance &&
           std::abs(data_.dock_depth_error) <= config_.align.depth_m_tolerance &&
           std::abs(data_.dock_sway_error) <= config_.align.sway_m_tolerance &&
           data_.dock_distance <= config_.align.dist_tolerance;
}

bool DockingController::readyForEnterDock() const
{
    return std::abs(data_.tag_yaw_error) <= config_.align_with_tag.yaw_deg_tolerance &&
           std::abs(data_.tag_depth_error) <= config_.align_with_tag.depth_m_tolerance &&
           std::abs(data_.tag_sway_error) <= config_.align_with_tag.sway_m_tolerance;
}

bool DockingController::tryCallModuleCommand(
    const std::string& module_name,
    const std::string& command,
    scheduler::ControlModuleResponse* response)
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

    if (response != nullptr)
    {
        *response = srv.response;
    }

    return true;
}

bool DockingController::stopModuleIfNeeded()
{
    if (!module_started_ || config_.autostart_module.empty())
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

    scheduler::ControlModuleResponse response;
    if (!tryCallModuleCommand(config_.autostart_module, "stop", &response))
    {
        return false;
    }

    if (response.success)
    {
        module_started_ = false;
        return true;
    }

    ROS_WARN_THROTTLE(
        5.0,
        "Docking module [%s] failed to stop: %s",
        config_.autostart_module.c_str(),
        response.message.c_str());
    return false;
}

DockingController::ControlCommand DockingController::buildCommand(State state) const
{
    const MotionLimitConfig* limits = nullptr;

    switch (state)
    {
    case State::SEARCH_BLUE_LIGHT:
        limits = &config_.search.command_limits;
        break;
    case State::APPROACH_BLUE_LIGHT:
        limits = &config_.approach.command_limits;
        break;
    case State::ALIGN_WITH_DOCK:
        limits = &config_.align.command_limits;
        break;
    case State::ALIGN_WITH_APRILTAG:
        limits = &config_.align_with_tag.command_limits;
        break;
    case State::ENTER_DOCK:
        limits = &config_.enter_dock.command_limits;
        break;
    case State::START:
    case State::WAIT_CAPTURE:
    case State::COMPLETED:
    case State::ERROR:
        return VehicleCommandInterface::Hold();
    }

    return VehicleCommandInterface::BodyVelocity(
        clampValue(data_.surge, -limits->surge, limits->surge),
        clampValue(data_.sway, -limits->sway, limits->sway),
        clampValue(data_.heave, -limits->heave, limits->heave),
        clampValue(data_.yaw, -limits->yaw, limits->yaw));
}
