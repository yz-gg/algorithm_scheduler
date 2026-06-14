#include <scheduler/pipeline_inspection_controller.h>

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

PipelineInspectionController::PipelineInspectionController(
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    VehicleCommandInterface* vehicle)
    : nh_(std::move(nh))
    , pnh_(std::move(pnh))
    , vehicle_(vehicle)
{
    loadParameters();

    observation_sub_ = nh_.subscribe<scheduler::PipelineCenterlineObservation>(
        config_.observation_topic,
        10,
        &PipelineInspectionController::observationCb,
        this);

    module_control_client_ = nh_.serviceClient<scheduler::ControlModule>(
        "/launch_manager_node/control_module");
}

void PipelineInspectionController::run()
{
    ros::Rate rate(config_.command_rate_hz);

    ROS_INFO("Pipeline inspection controller started");

    if (config_.auto_start_inspection)
    {
        enterState(
            InspectionState::STARTING_INSPECTION,
            "inspection state machine booted");
    }
    else
    {
        enterState(
            InspectionState::SEARCH_PRIMARY_LINE,
            "auto start disabled, waiting for primary line");
    }

    while (ros::ok())
    {
        ros::spinOnce();
        stepStateMachine();
        rate.sleep();
    }

    vehicle_->publishCommand(VehicleCommandInterface::RcRelease());
}

std::string PipelineInspectionController::missionType() const
{
    return "pipeline_inspection";
}

void PipelineInspectionController::loadParameters()
{
    pnh_.param<std::string>(
        "autostart_module", config_.autostart_module, config_.autostart_module);
    pnh_.param<std::string>(
        "observation_topic",
        config_.observation_topic,
        config_.observation_topic);

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
        "line_stable_hold_sec",
        config_.line_stable_hold_sec,
        config_.line_stable_hold_sec);
    pnh_.param("align_hold_sec", config_.align_hold_sec, config_.align_hold_sec);
    pnh_.param(
        "turn_trigger_hold_sec",
        config_.turn_trigger_hold_sec,
        config_.turn_trigger_hold_sec);
    pnh_.param(
        "turn_commit_sec",
        config_.turn_commit_sec,
        config_.turn_commit_sec);
    pnh_.param("turn_timeout_sec", config_.turn_timeout_sec, config_.turn_timeout_sec);
    pnh_.param(
        "recovery_timeout_sec",
        config_.recovery_timeout_sec,
        config_.recovery_timeout_sec);
    pnh_.param(
        "search_reverse_interval_sec",
        config_.search_reverse_interval_sec,
        config_.search_reverse_interval_sec);
    pnh_.param(
        "inspection_duration_sec",
        config_.inspection_duration_sec,
        config_.inspection_duration_sec);
    pnh_.param(
        "min_observation_confidence",
        config_.min_observation_confidence,
        config_.min_observation_confidence);

    pnh_.param(
        "min_primary_line_points",
        config_.min_primary_line_points,
        config_.min_primary_line_points);
    pnh_.param(
        "min_secondary_line_points",
        config_.min_secondary_line_points,
        config_.min_secondary_line_points);

    pnh_.param(
        "reference_row_ratio",
        config_.reference_row_ratio,
        config_.reference_row_ratio);
    pnh_.param(
        "lateral_tolerance_norm",
        config_.lateral_tolerance_norm,
        config_.lateral_tolerance_norm);
    pnh_.param(
        "angle_tolerance_rad",
        config_.angle_tolerance_rad,
        config_.angle_tolerance_rad);

    pnh_.param(
        "sway_from_lateral_gain",
        config_.sway_from_lateral_gain,
        config_.sway_from_lateral_gain);
    pnh_.param(
        "sway_from_lateral_rate_gain",
        config_.sway_from_lateral_rate_gain,
        config_.sway_from_lateral_rate_gain);
    pnh_.param(
        "yaw_rate_from_angle_gain",
        config_.yaw_rate_from_angle_gain,
        config_.yaw_rate_from_angle_gain);
    pnh_.param(
        "yaw_rate_from_lateral_gain",
        config_.yaw_rate_from_lateral_gain,
        config_.yaw_rate_from_lateral_gain);

    pnh_.param(
        "align_surge_command",
        config_.align_surge_command,
        config_.align_surge_command);
    pnh_.param(
        "track_surge_command",
        config_.track_surge_command,
        config_.track_surge_command);
    pnh_.param(
        "turn_surge_command",
        config_.turn_surge_command,
        config_.turn_surge_command);
    pnh_.param(
        "search_yaw_command",
        config_.search_yaw_command,
        config_.search_yaw_command);
    pnh_.param(
        "recovery_yaw_command",
        config_.recovery_yaw_command,
        config_.recovery_yaw_command);
    pnh_.param("turn_yaw_rate", config_.turn_yaw_rate, config_.turn_yaw_rate);

    pnh_.param(
        "max_forward_speed", config_.max_forward_speed, config_.max_forward_speed);
    pnh_.param(
        "max_lateral_speed", config_.max_lateral_speed, config_.max_lateral_speed);
    pnh_.param("max_yaw_rate", config_.max_yaw_rate, config_.max_yaw_rate);

    pnh_.param(
        "error_filter_tau_sec",
        config_.error_filter_tau_sec,
        config_.error_filter_tau_sec);
    pnh_.param(
        "error_rate_filter_tau_sec",
        config_.error_rate_filter_tau_sec,
        config_.error_rate_filter_tau_sec);
    pnh_.param(
        "minimum_surge_scale",
        config_.minimum_surge_scale,
        config_.minimum_surge_scale);
    pnh_.param(
        "surge_lateral_error_weight",
        config_.surge_lateral_error_weight,
        config_.surge_lateral_error_weight);
    pnh_.param(
        "surge_angle_error_weight",
        config_.surge_angle_error_weight,
        config_.surge_angle_error_weight);
    pnh_.param(
        "hard_stop_lateral_error",
        config_.hard_stop_lateral_error,
        config_.hard_stop_lateral_error);
    pnh_.param(
        "hard_stop_angle_error",
        config_.hard_stop_angle_error,
        config_.hard_stop_angle_error);

    pnh_.param(
        "medium_gear_lateral_error",
        config_.medium_gear_lateral_error,
        config_.medium_gear_lateral_error);
    pnh_.param(
        "medium_gear_angle_error",
        config_.medium_gear_angle_error,
        config_.medium_gear_angle_error);
    pnh_.param(
        "fast_gear_lateral_error",
        config_.fast_gear_lateral_error,
        config_.fast_gear_lateral_error);
    pnh_.param(
        "fast_gear_angle_error",
        config_.fast_gear_angle_error,
        config_.fast_gear_angle_error);
    pnh_.param(
        "fast_gear_min_confidence",
        config_.fast_gear_min_confidence,
        config_.fast_gear_min_confidence);

    pnh_.param(
        "yaw_rate_feedback_kp",
        config_.yaw_rate_feedback_kp,
        config_.yaw_rate_feedback_kp);
    pnh_.param(
        "linear_velocity_feedback_kp",
        config_.linear_velocity_feedback_kp,
        config_.linear_velocity_feedback_kp);
    pnh_.param(
        "min_linear_velocity_quality",
        config_.min_linear_velocity_quality,
        config_.min_linear_velocity_quality);
    pnh_.param(
        "feedback_timeout_sec",
        config_.feedback_timeout_sec,
        config_.feedback_timeout_sec);

    pnh_.param(
        "corner_approach_bottom_distance_px",
        config_.corner_approach_bottom_distance_px,
        config_.corner_approach_bottom_distance_px);
    pnh_.param(
        "turn_trigger_bottom_distance_px",
        config_.turn_trigger_bottom_distance_px,
        config_.turn_trigger_bottom_distance_px);
    pnh_.param(
        "turn_direction_sign_from_image_x",
        config_.turn_direction_sign_from_image_x,
        config_.turn_direction_sign_from_image_x);

    pnh_.param(
        "module_request_interval_sec",
        config_.module_request_interval_sec,
        config_.module_request_interval_sec);
    pnh_.param(
        "service_wait_sec", config_.service_wait_sec, config_.service_wait_sec);

    config_.command_rate_hz = std::max(1.0, config_.command_rate_hz);
    config_.min_observation_confidence = clampValue(
        config_.min_observation_confidence, 0.0, 1.0);
    config_.minimum_surge_scale = clampValue(
        config_.minimum_surge_scale, 0.0, 1.0);
    config_.align_surge_command = clampValue(
        config_.align_surge_command, 0.0, 1.0);
    config_.track_surge_command = clampValue(
        config_.track_surge_command, 0.0, 1.0);
    config_.turn_surge_command = clampValue(
        config_.turn_surge_command, 0.0, 1.0);
    config_.corner_approach_bottom_distance_px = std::max(
        config_.corner_approach_bottom_distance_px,
        config_.turn_trigger_bottom_distance_px);
}

void PipelineInspectionController::observationCb(
    const scheduler::PipelineCenterlineObservation::ConstPtr& msg)
{
    last_observation_.detected = msg->detected;
    last_observation_.has_secondary_line = msg->has_secondary_line;
    last_observation_.has_intersection = msg->has_intersection;
    last_observation_.confidence = msg->confidence;
    last_observation_.range_hint_m = msg->range_hint_m;
    last_observation_.image_width = msg->image_width;
    last_observation_.image_height = msg->image_height;
    last_observation_.primary_line_points.clear();
    last_observation_.secondary_line_points.clear();

    for (const auto& point : msg->primary_line_points)
    {
        last_observation_.primary_line_points.push_back(
            ImagePoint{static_cast<double>(point.x), static_cast<double>(point.y)});
    }

    for (const auto& point : msg->secondary_line_points)
    {
        last_observation_.secondary_line_points.push_back(
            ImagePoint{static_cast<double>(point.x), static_cast<double>(point.y)});
    }

    last_observation_.intersection_point = ImagePoint{
        static_cast<double>(msg->intersection_point.x),
        static_cast<double>(msg->intersection_point.y)};

    last_observation_.stamp =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
}

void PipelineInspectionController::stepStateMachine()
{
    switch (state_)
    {
    case InspectionState::STARTING_INSPECTION:
        handleStartInspection();
        break;
    case InspectionState::SEARCH_PRIMARY_LINE:
        handleSearchPrimaryLine();
        break;
    case InspectionState::ALIGN_PRIMARY_LINE:
        handleAlignPrimaryLine();
        break;
    case InspectionState::TRACK_PRIMARY_LINE:
        handleTrackPrimaryLine();
        break;
    case InspectionState::APPROACH_CORNER:
        handleApproachCorner();
        break;
    case InspectionState::TURNING_TO_SECONDARY_LINE:
        handleTurningToSecondaryLine();
        break;
    case InspectionState::ALIGN_SECONDARY_LINE:
        handleAlignSecondaryLine();
        break;
    case InspectionState::TRACK_SECONDARY_LINE:
        handleTrackSecondaryLine();
        break;
    case InspectionState::RECOVER_LINE:
        handleRecoverLine();
        break;
    case InspectionState::FINISHING:
        handleFinishing();
        break;
    case InspectionState::COMPLETED:
        vehicle_->publishCommand(VehicleCommandInterface::RcRelease());
        break;
    case InspectionState::FAULT:
    case InspectionState::BOOTSTRAP:
    default:
        vehicle_->publishCommand(VehicleCommandInterface::RcNeutral());
        break;
    }
}

void PipelineInspectionController::enterState(
    InspectionState next_state,
    const std::string& reason)
{
    if (state_ == next_state)
    {
        return;
    }

    ROS_INFO_STREAM(
        "Inspection state " << stateToString(state_) << " -> "
                            << stateToString(next_state) << " (" << reason << ")");

    state_ = next_state;
    state_entered_at_ = ros::Time::now();
    line_stable_since_ = ros::Time();
    align_stable_since_ = ros::Time();
    turn_ready_since_ = ros::Time();

    if (next_state == InspectionState::SEARCH_PRIMARY_LINE ||
        next_state == InspectionState::RECOVER_LINE)
    {
        error_filter_initialized_ = false;
        last_error_update_time_ = ros::Time();
    }

    if (next_state == InspectionState::TURNING_TO_SECONDARY_LINE)
    {
        turn_started_at_ = ros::Time::now();
    }

    if (next_state == InspectionState::TRACK_PRIMARY_LINE ||
        next_state == InspectionState::TRACK_SECONDARY_LINE)
    {
        if (tracking_started_at_.isZero())
        {
            tracking_started_at_ = ros::Time::now();
        }
    }
}

std::string PipelineInspectionController::stateToString(
    InspectionState state) const
{
    switch (state)
    {
    case InspectionState::BOOTSTRAP:
        return "BOOTSTRAP";
    case InspectionState::STARTING_INSPECTION:
        return "STARTING_INSPECTION";
    case InspectionState::SEARCH_PRIMARY_LINE:
        return "SEARCH_PRIMARY_LINE";
    case InspectionState::ALIGN_PRIMARY_LINE:
        return "ALIGN_PRIMARY_LINE";
    case InspectionState::TRACK_PRIMARY_LINE:
        return "TRACK_PRIMARY_LINE";
    case InspectionState::APPROACH_CORNER:
        return "APPROACH_CORNER";
    case InspectionState::TURNING_TO_SECONDARY_LINE:
        return "TURNING_TO_SECONDARY_LINE";
    case InspectionState::ALIGN_SECONDARY_LINE:
        return "ALIGN_SECONDARY_LINE";
    case InspectionState::TRACK_SECONDARY_LINE:
        return "TRACK_SECONDARY_LINE";
    case InspectionState::RECOVER_LINE:
        return "RECOVER_LINE";
    case InspectionState::FINISHING:
        return "FINISHING";
    case InspectionState::COMPLETED:
        return "COMPLETED";
    case InspectionState::FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

void PipelineInspectionController::handleStartInspection()
{
    vehicle_->publishCommand(VehicleCommandInterface::RcNeutral());

    if (config_.autostart_module.empty())
    {
        enterState(
            InspectionState::SEARCH_PRIMARY_LINE,
            "no autostart module configured");
        return;
    }

    if (module_started_)
    {
        enterState(
            InspectionState::SEARCH_PRIMARY_LINE,
            "inspection module already running");
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
        enterState(
            InspectionState::SEARCH_PRIMARY_LINE,
            "inspection module started");
        return;
    }

    ROS_WARN_THROTTLE(
        5.0,
        "Inspection module [%s] failed to start: %s",
        config_.autostart_module.c_str(),
        response.message.c_str());
}

void PipelineInspectionController::handleSearchPrimaryLine()
{
    if (!hasUsablePrimaryLine())
    {
        vehicle_->publishCommand(buildSearchCommand());
        line_stable_since_ = ros::Time();
        return;
    }

    vehicle_->publishCommand(VehicleCommandInterface::RcNeutral());

    if (line_stable_since_.isZero())
    {
        line_stable_since_ = ros::Time::now();
        return;
    }

    if ((ros::Time::now() - line_stable_since_).toSec() >= config_.line_stable_hold_sec)
    {
        enterState(InspectionState::ALIGN_PRIMARY_LINE, "primary line is stable");
    }
}

void PipelineInspectionController::handleAlignPrimaryLine()
{
    if (!hasUsablePrimaryLine())
    {
        beginRecovery(
            RecoveryTarget::PRIMARY,
            "lost primary line while aligning");
        return;
    }

    vehicle_->publishCommand(buildTrackingCommand(TargetLine::PRIMARY, false));

    if (!isTargetAligned(TargetLine::PRIMARY))
    {
        align_stable_since_ = ros::Time();
        return;
    }

    if (align_stable_since_.isZero())
    {
        align_stable_since_ = ros::Time::now();
        return;
    }

    if ((ros::Time::now() - align_stable_since_).toSec() >= config_.align_hold_sec)
    {
        enterState(InspectionState::TRACK_PRIMARY_LINE, "primary line aligned");
    }
}

void PipelineInspectionController::handleTrackPrimaryLine()
{
    if (!hasUsablePrimaryLine())
    {
        beginRecovery(
            RecoveryTarget::PRIMARY,
            "lost primary line while tracking");
        return;
    }

    if (config_.inspection_duration_sec > 0.0 && !tracking_started_at_.isZero() &&
        (ros::Time::now() - tracking_started_at_).toSec() >=
            config_.inspection_duration_sec)
    {
        enterState(InspectionState::FINISHING, "inspection duration reached");
        return;
    }

    if (!hasUsableSecondaryLine())
    {
        vehicle_->publishCommand(buildTrackingCommand(TargetLine::PRIMARY, true));
        return;
    }

    double bottom_distance_px = 0.0;
    if (!getIntersectionBottomDistancePx(&bottom_distance_px))
    {
        vehicle_->publishCommand(buildTrackingCommand(TargetLine::PRIMARY, true));
        return;
    }

    if (bottom_distance_px <= config_.corner_approach_bottom_distance_px)
    {
        enterState(
            InspectionState::APPROACH_CORNER,
            "secondary line and corner entered approach range");
        vehicle_->publishCommand(buildTrackingCommand(TargetLine::PRIMARY, false));
        return;
    }

    vehicle_->publishCommand(buildTrackingCommand(TargetLine::PRIMARY, true));
}

void PipelineInspectionController::handleApproachCorner()
{
    if (!hasUsablePrimaryLine())
    {
        beginRecovery(
            RecoveryTarget::PRIMARY,
            "lost primary line while approaching corner");
        return;
    }

    if (!hasUsableSecondaryLine())
    {
        enterState(
            InspectionState::TRACK_PRIMARY_LINE,
            "secondary line disappeared before corner");
        return;
    }

    double bottom_distance_px = 0.0;
    if (!getIntersectionBottomDistancePx(&bottom_distance_px))
    {
        enterState(
            InspectionState::TRACK_PRIMARY_LINE,
            "corner intersection disappeared");
        return;
    }

    vehicle_->publishCommand(buildTrackingCommand(TargetLine::PRIMARY, false));

    if (bottom_distance_px > config_.turn_trigger_bottom_distance_px)
    {
        turn_ready_since_ = ros::Time();
        return;
    }

    if (turn_ready_since_.isZero())
    {
        turn_ready_since_ = ros::Time::now();
        return;
    }

    if ((ros::Time::now() - turn_ready_since_).toSec() < config_.turn_trigger_hold_sec)
    {
        return;
    }

    if (!resolveTurnDirection(&turn_direction_sign_))
    {
        ROS_WARN_THROTTLE(2.0, "Turn trigger reached but turn direction is ambiguous");
        return;
    }

    turn_direction_ready_ = true;
    enterState(
        InspectionState::TURNING_TO_SECONDARY_LINE,
        "corner is close enough, start yaw turn");
}

void PipelineInspectionController::handleTurningToSecondaryLine()
{
    if (!turn_direction_ready_)
    {
        enterState(InspectionState::FAULT, "turn direction not available");
        return;
    }

    vehicle_->publishCommand(buildTurningCommand());

    const double turn_elapsed_sec =
        turn_started_at_.isZero() ? 0.0 : (ros::Time::now() - turn_started_at_).toSec();

    if (turn_elapsed_sec >= config_.turn_commit_sec && hasUsableSecondaryLine())
    {
        enterState(
            InspectionState::ALIGN_SECONDARY_LINE,
            "secondary line found after committed turn");
        return;
    }

    if (!turn_started_at_.isZero() &&
        (ros::Time::now() - turn_started_at_).toSec() >= config_.turn_timeout_sec)
    {
        beginRecovery(
            RecoveryTarget::SECONDARY,
            "secondary line was not found before turn timeout");
    }
}

void PipelineInspectionController::handleAlignSecondaryLine()
{
    if (!hasUsableSecondaryLine())
    {
        beginRecovery(
            RecoveryTarget::SECONDARY,
            "lost secondary line while aligning");
        return;
    }

    vehicle_->publishCommand(buildTrackingCommand(TargetLine::SECONDARY, false));

    if (!isTargetAligned(TargetLine::SECONDARY))
    {
        align_stable_since_ = ros::Time();
        return;
    }

    if (align_stable_since_.isZero())
    {
        align_stable_since_ = ros::Time::now();
        return;
    }

    if ((ros::Time::now() - align_stable_since_).toSec() >= config_.align_hold_sec)
    {
        enterState(InspectionState::TRACK_SECONDARY_LINE, "secondary line aligned");
    }
}

void PipelineInspectionController::handleTrackSecondaryLine()
{
    if (!hasUsableSecondaryLine())
    {
        beginRecovery(
            RecoveryTarget::SECONDARY,
            "lost secondary line while tracking");
        return;
    }

    if (config_.inspection_duration_sec > 0.0 && !tracking_started_at_.isZero() &&
        (ros::Time::now() - tracking_started_at_).toSec() >=
            config_.inspection_duration_sec)
    {
        enterState(InspectionState::FINISHING, "inspection duration reached");
        return;
    }

    vehicle_->publishCommand(buildTrackingCommand(TargetLine::SECONDARY, true));
}

void PipelineInspectionController::handleRecoverLine()
{
    const bool recovered = recovery_target_ == RecoveryTarget::PRIMARY
        ? hasUsablePrimaryLine()
        : hasUsableSecondaryLine();

    if (recovered)
    {
        vehicle_->publishCommand(VehicleCommandInterface::RcNeutral());

        if (line_stable_since_.isZero())
        {
            line_stable_since_ = ros::Time::now();
        }
        else if ((ros::Time::now() - line_stable_since_).toSec() >=
                 config_.line_stable_hold_sec)
        {
            enterState(
                recovery_target_ == RecoveryTarget::PRIMARY
                    ? InspectionState::ALIGN_PRIMARY_LINE
                    : InspectionState::ALIGN_SECONDARY_LINE,
                "target line recovered");
            return;
        }
    }
    else
    {
        line_stable_since_ = ros::Time();
        vehicle_->publishCommand(buildRecoveryCommand());
    }

    if (!state_entered_at_.isZero() &&
        (ros::Time::now() - state_entered_at_).toSec() >=
            config_.recovery_timeout_sec)
    {
        if (recovery_target_ == RecoveryTarget::PRIMARY)
        {
            enterState(
                InspectionState::SEARCH_PRIMARY_LINE,
                "primary line recovery timed out");
        }
        else
        {
            enterState(
                InspectionState::FAULT,
                "secondary line recovery timed out");
        }
    }
}

void PipelineInspectionController::beginRecovery(
    RecoveryTarget target,
    const std::string& reason)
{
    recovery_target_ = target;
    recovery_scan_sign_ = turn_direction_ready_ ? turn_direction_sign_ : 1.0;
    vehicle_->publishCommand(VehicleCommandInterface::RcNeutral());
    enterState(InspectionState::RECOVER_LINE, reason);
}

void PipelineInspectionController::handleFinishing()
{
    vehicle_->publishCommand(VehicleCommandInterface::RcNeutral());

    if (!module_started_ || config_.autostart_module.empty())
    {
        enterState(InspectionState::COMPLETED, "inspection stopped");
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
    if (!tryCallModuleCommand(config_.autostart_module, "stop", &response))
    {
        return;
    }

    if (response.success)
    {
        module_started_ = false;
        enterState(InspectionState::COMPLETED, "inspection module stopped");
        return;
    }

    ROS_WARN_THROTTLE(
        5.0,
        "Inspection module [%s] failed to stop: %s",
        config_.autostart_module.c_str(),
        response.message.c_str());
}

bool PipelineInspectionController::hasFreshObservation() const
{
    if (!last_observation_.detected)
    {
        return false;
    }

    if (last_observation_.confidence < config_.min_observation_confidence)
    {
        return false;
    }

    if (last_observation_.image_width == 0 || last_observation_.image_height == 0)
    {
        return false;
    }

    if (last_observation_.stamp.isZero())
    {
        return false;
    }

    return (ros::Time::now() - last_observation_.stamp).toSec() <=
           config_.observation_timeout_sec;
}

bool PipelineInspectionController::hasUsablePrimaryLine() const
{
    FittedLine fitted_line;
    return getTargetLine(TargetLine::PRIMARY, &fitted_line);
}

bool PipelineInspectionController::hasUsableSecondaryLine() const
{
    FittedLine fitted_line;
    return getTargetLine(TargetLine::SECONDARY, &fitted_line);
}

bool PipelineInspectionController::tryCallModuleCommand(
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

bool PipelineInspectionController::fitLinePoints(
    const std::vector<ImagePoint>& points,
    int min_points,
    FittedLine* fitted_line) const
{
    if (fitted_line == nullptr)
    {
        return false;
    }

    *fitted_line = FittedLine{};

    if (!hasFreshObservation() || static_cast<int>(points.size()) < min_points)
    {
        return false;
    }

    double mean_x = 0.0;
    double mean_y = 0.0;
    for (const auto& point : points)
    {
        mean_x += point.x;
        mean_y += point.y;
    }

    mean_x /= static_cast<double>(points.size());
    mean_y /= static_cast<double>(points.size());

    double sxx = 0.0;
    double syy = 0.0;
    double sxy = 0.0;
    for (const auto& point : points)
    {
        const double dx = point.x - mean_x;
        const double dy = point.y - mean_y;
        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }

    const double angle = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    double dir_x = std::cos(angle);
    double dir_y = std::sin(angle);

    if (dir_y < 0.0)
    {
        dir_x = -dir_x;
        dir_y = -dir_y;
    }

    fitted_line->valid = true;
    fitted_line->center_x = mean_x;
    fitted_line->center_y = mean_y;
    fitted_line->dx = dir_x;
    fitted_line->dy = dir_y;
    fitted_line->point_count = points.size();
    return true;
}

bool PipelineInspectionController::getTargetLine(
    TargetLine target,
    FittedLine* fitted_line) const
{
    if (target == TargetLine::PRIMARY)
    {
        return fitLinePoints(
            last_observation_.primary_line_points,
            config_.min_primary_line_points,
            fitted_line);
    }

    if (!last_observation_.has_secondary_line)
    {
        return false;
    }

    return fitLinePoints(
        last_observation_.secondary_line_points,
        config_.min_secondary_line_points,
        fitted_line);
}

double PipelineInspectionController::getReferenceRowPx() const
{
    const double height = static_cast<double>(last_observation_.image_height);
    return clampValue(
        config_.reference_row_ratio * height,
        0.0,
        std::max(0.0, height - 1.0));
}

bool PipelineInspectionController::computeTargetErrors(
    TargetLine target,
    double* lateral_error_norm,
    double* angle_error_rad) const
{
    FittedLine fitted_line;
    if (!getTargetLine(target, &fitted_line))
    {
        return false;
    }

    const double width = static_cast<double>(last_observation_.image_width);
    if (width <= 1.0)
    {
        return false;
    }

    const double reference_y = getReferenceRowPx();
    double reference_x = fitted_line.center_x;

    if (std::fabs(fitted_line.dy) > 1e-6)
    {
        const double slope_x_over_y = fitted_line.dx / fitted_line.dy;
        reference_x =
            fitted_line.center_x + slope_x_over_y * (reference_y - fitted_line.center_y);
    }

    if (lateral_error_norm != nullptr)
    {
        *lateral_error_norm = (reference_x - 0.5 * width) / (0.5 * width);
    }

    if (angle_error_rad != nullptr)
    {
        *angle_error_rad = std::atan2(fitted_line.dx, fitted_line.dy);
    }

    return true;
}

bool PipelineInspectionController::isTargetAligned(TargetLine target) const
{
    double lateral_error_norm = 0.0;
    double angle_error_rad = 0.0;
    if (!computeTargetErrors(target, &lateral_error_norm, &angle_error_rad))
    {
        return false;
    }

    return std::fabs(lateral_error_norm) <= config_.lateral_tolerance_norm &&
           std::fabs(angle_error_rad) <= config_.angle_tolerance_rad;
}

bool PipelineInspectionController::getIntersectionBottomDistancePx(
    double* bottom_distance_px) const
{
    if (!hasFreshObservation() || !last_observation_.has_intersection)
    {
        return false;
    }

    if (bottom_distance_px == nullptr)
    {
        return false;
    }

    *bottom_distance_px =
        static_cast<double>(last_observation_.image_height) -
        last_observation_.intersection_point.y;
    return true;
}

bool PipelineInspectionController::resolveTurnDirection(
    double* turn_direction_sign) const
{
    if (turn_direction_sign == nullptr || !hasUsableSecondaryLine() ||
        !last_observation_.has_intersection)
    {
        return false;
    }

    double mean_x = 0.0;
    for (const auto& point : last_observation_.secondary_line_points)
    {
        mean_x += point.x;
    }

    mean_x /= static_cast<double>(last_observation_.secondary_line_points.size());

    const double dx_from_intersection = mean_x - last_observation_.intersection_point.x;
    if (std::fabs(dx_from_intersection) < 1.0)
    {
        return false;
    }

    const double image_direction = dx_from_intersection > 0.0 ? 1.0 : -1.0;
    *turn_direction_sign =
        image_direction * config_.turn_direction_sign_from_image_x;
    return true;
}

PipelineInspectionController::ControlCommand
PipelineInspectionController::buildTrackingCommand(
    TargetLine target,
    bool tracking_mode)
{
    ControlErrors errors;
    if (!updateControlErrors(target, &errors))
    {
        return VehicleCommandInterface::RcNeutral();
    }

    MotionReference reference;
    reference.sway = clampValue(
        config_.sway_from_lateral_gain * errors.lateral +
            config_.sway_from_lateral_rate_gain * errors.lateral_rate,
        -1.0,
        1.0);

    reference.yaw_rate = clampValue(
        config_.yaw_rate_from_angle_gain * errors.angle +
            config_.yaw_rate_from_lateral_gain * errors.lateral,
        -config_.max_yaw_rate,
        config_.max_yaw_rate);

    const double alignment_scale = std::exp(
        -config_.surge_lateral_error_weight * std::fabs(errors.lateral) -
        config_.surge_angle_error_weight * std::fabs(errors.angle));
    const double confidence_denominator =
        std::max(1e-6, 1.0 - config_.min_observation_confidence);
    const double confidence_scale = clampValue(
        (last_observation_.confidence - config_.min_observation_confidence) /
            confidence_denominator,
        config_.minimum_surge_scale,
        1.0);

    const double base_surge = tracking_mode
        ? config_.track_surge_command
        : config_.align_surge_command;
    reference.surge = clampValue(
        base_surge *
            std::max(
                config_.minimum_surge_scale,
                alignment_scale * confidence_scale),
        0.0,
        1.0);

    if (std::fabs(errors.lateral) >= config_.hard_stop_lateral_error ||
        std::fabs(errors.angle) >= config_.hard_stop_angle_error)
    {
        reference.surge = 0.0;
    }

    reference.gear = selectTrackingGear(errors, tracking_mode);
    return buildRcCommand(reference);
}

PipelineInspectionController::ControlCommand
PipelineInspectionController::buildTurningCommand()
{
    MotionReference reference;
    reference.surge = clampValue(config_.turn_surge_command, 0.0, 1.0);
    reference.yaw_rate = turn_direction_sign_ * config_.turn_yaw_rate;
    reference.gear = RcSpeedGear::SLOW;

    ControlErrors errors;
    if (hasUsableSecondaryLine() &&
        updateControlErrors(TargetLine::SECONDARY, &errors))
    {
        const double visual_yaw_rate = clampValue(
            config_.yaw_rate_from_angle_gain * errors.angle +
                config_.yaw_rate_from_lateral_gain * errors.lateral,
            -config_.max_yaw_rate,
            config_.max_yaw_rate);
        const double minimum_turn_rate = 0.35 * config_.turn_yaw_rate;
        reference.yaw_rate = visual_yaw_rate * turn_direction_sign_ >=
                minimum_turn_rate
            ? visual_yaw_rate
            : turn_direction_sign_ * minimum_turn_rate;
    }

    return buildRcCommand(reference);
}

PipelineInspectionController::ControlCommand
PipelineInspectionController::buildSearchCommand() const
{
    const double interval = std::max(0.1, config_.search_reverse_interval_sec);
    const double elapsed = state_entered_at_.isZero()
        ? 0.0
        : (ros::Time::now() - state_entered_at_).toSec();
    const int phase = static_cast<int>(elapsed / interval);
    const double direction = phase % 2 == 0 ? 1.0 : -1.0;

    return VehicleCommandInterface::RcOverride(
        0.0,
        0.0,
        0.0,
        direction * clampValue(config_.search_yaw_command, 0.0, 1.0),
        RcSpeedGear::SLOW);
}

PipelineInspectionController::ControlCommand
PipelineInspectionController::buildRecoveryCommand() const
{
    const double elapsed = state_entered_at_.isZero()
        ? 0.0
        : (ros::Time::now() - state_entered_at_).toSec();
    const double half_timeout = 0.5 * std::max(0.1, config_.recovery_timeout_sec);
    const double sweep_sign = elapsed < half_timeout ? 1.0 : -1.0;

    return VehicleCommandInterface::RcOverride(
        0.0,
        0.0,
        0.0,
        sweep_sign * recovery_scan_sign_ *
            clampValue(config_.recovery_yaw_command, 0.0, 1.0),
        RcSpeedGear::SLOW);
}

PipelineInspectionController::ControlCommand
PipelineInspectionController::buildRcCommand(
    const MotionReference& reference) const
{
    double surge = clampValue(reference.surge, -1.0, 1.0);
    double sway = clampValue(reference.sway, -1.0, 1.0);
    double heave = clampValue(reference.heave, -1.0, 1.0);
    double yaw = config_.max_yaw_rate > 1e-6
        ? reference.yaw_rate / config_.max_yaw_rate
        : 0.0;

    VehicleCommandInterface::Feedback feedback;
    if (getFreshFeedback(&feedback))
    {
        if (feedback.angular_velocity_valid && config_.max_yaw_rate > 1e-6)
        {
            yaw += config_.yaw_rate_feedback_kp *
                (reference.yaw_rate - feedback.yaw_rate) /
                config_.max_yaw_rate;
        }

        if (feedback.linear_velocity_valid &&
            feedback.linear_velocity_quality >=
                config_.min_linear_velocity_quality)
        {
            if (config_.max_forward_speed > 1e-6)
            {
                const double desired_vx =
                    reference.surge * config_.max_forward_speed;
                surge += config_.linear_velocity_feedback_kp *
                    (desired_vx - feedback.body_vx) /
                    config_.max_forward_speed;
            }

            if (config_.max_lateral_speed > 1e-6)
            {
                const double desired_vy =
                    reference.sway * config_.max_lateral_speed;
                sway += config_.linear_velocity_feedback_kp *
                    (desired_vy - feedback.body_vy) /
                    config_.max_lateral_speed;
            }
        }
    }

    return VehicleCommandInterface::RcOverride(
        clampValue(surge, -1.0, 1.0),
        clampValue(sway, -1.0, 1.0),
        clampValue(heave, -1.0, 1.0),
        clampValue(yaw, -1.0, 1.0),
        reference.gear);
}

bool PipelineInspectionController::updateControlErrors(
    TargetLine target,
    ControlErrors* errors)
{
    if (errors == nullptr)
    {
        return false;
    }

    double raw_lateral = 0.0;
    double raw_angle = 0.0;
    if (!computeTargetErrors(target, &raw_lateral, &raw_angle))
    {
        return false;
    }

    const ros::Time now = ros::Time::now();
    if (!error_filter_initialized_ || error_filter_target_ != target ||
        last_error_update_time_.isZero())
    {
        error_filter_initialized_ = true;
        error_filter_target_ = target;
        filtered_errors_.lateral = raw_lateral;
        filtered_errors_.angle = raw_angle;
        filtered_errors_.lateral_rate = 0.0;
        last_error_update_time_ = now;
        *errors = filtered_errors_;
        return true;
    }

    const double dt = std::max(1e-3, (now - last_error_update_time_).toSec());
    const double error_alpha = clampValue(
        dt / (std::max(0.0, config_.error_filter_tau_sec) + dt),
        0.0,
        1.0);
    const double old_lateral = filtered_errors_.lateral;

    filtered_errors_.lateral += error_alpha *
        (raw_lateral - filtered_errors_.lateral);
    filtered_errors_.angle += error_alpha *
        (raw_angle - filtered_errors_.angle);

    const double raw_lateral_rate =
        (filtered_errors_.lateral - old_lateral) / dt;
    const double rate_alpha = clampValue(
        dt / (std::max(0.0, config_.error_rate_filter_tau_sec) + dt),
        0.0,
        1.0);
    filtered_errors_.lateral_rate += rate_alpha *
        (raw_lateral_rate - filtered_errors_.lateral_rate);

    last_error_update_time_ = now;
    *errors = filtered_errors_;
    return true;
}

RcSpeedGear PipelineInspectionController::selectTrackingGear(
    const ControlErrors& errors,
    bool tracking_mode) const
{
    if (!tracking_mode)
    {
        return RcSpeedGear::SLOW;
    }

    if (std::fabs(errors.lateral) <= config_.fast_gear_lateral_error &&
        std::fabs(errors.angle) <= config_.fast_gear_angle_error &&
        last_observation_.confidence >= config_.fast_gear_min_confidence)
    {
        return RcSpeedGear::FAST;
    }

    if (std::fabs(errors.lateral) <= config_.medium_gear_lateral_error &&
        std::fabs(errors.angle) <= config_.medium_gear_angle_error)
    {
        return RcSpeedGear::MEDIUM;
    }

    return RcSpeedGear::SLOW;
}

bool PipelineInspectionController::getFreshFeedback(
    VehicleCommandInterface::Feedback* feedback) const
{
    if (feedback == nullptr || !vehicle_->getFeedback(feedback) ||
        feedback->stamp.isZero())
    {
        return false;
    }

    return (ros::Time::now() - feedback->stamp).toSec() <=
        config_.feedback_timeout_sec;
}
