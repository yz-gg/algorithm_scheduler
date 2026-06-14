#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <scheduler/ControlModule.h>
#include <scheduler/PipelineCenterlineObservation.h>
#include <scheduler/mission_controller.h>
#include <scheduler/vehicle_command_interface.h>

class PipelineInspectionController : public MissionController
{
public:
    PipelineInspectionController(
        ros::NodeHandle nh,
        ros::NodeHandle pnh,
        VehicleCommandInterface* vehicle);

    void run() override;
    std::string missionType() const override;

private:
    enum class InspectionState
    {
        BOOTSTRAP,
        STARTING_INSPECTION,
        SEARCH_PRIMARY_LINE,
        ALIGN_PRIMARY_LINE,
        TRACK_PRIMARY_LINE,
        APPROACH_CORNER,
        TURNING_TO_SECONDARY_LINE,
        ALIGN_SECONDARY_LINE,
        TRACK_SECONDARY_LINE,
        RECOVER_LINE,
        FINISHING,
        COMPLETED,
        FAULT,
    };

    enum class TargetLine
    {
        PRIMARY,
        SECONDARY,
    };

    enum class RecoveryTarget
    {
        PRIMARY,
        SECONDARY,
    };

    struct ImagePoint
    {
        ImagePoint() = default;
        ImagePoint(double x_value, double y_value)
            : x(x_value)
            , y(y_value)
        {
        }

        double x{0.0};
        double y{0.0};
    };

    struct FittedLine
    {
        bool valid{false};
        double center_x{0.0};
        double center_y{0.0};
        double dx{0.0};
        double dy{1.0};
        std::size_t point_count{0};
    };

    struct Observation
    {
        bool detected{false};
        bool has_secondary_line{false};
        bool has_intersection{false};
        double confidence{0.0};
        double range_hint_m{-1.0};
        std::uint32_t image_width{0};
        std::uint32_t image_height{0};
        std::vector<ImagePoint> primary_line_points;
        std::vector<ImagePoint> secondary_line_points;
        ImagePoint intersection_point;
        ros::Time stamp;
    };

    struct ControlErrors
    {
        double lateral{0.0};
        double angle{0.0};
        double lateral_rate{0.0};
    };

    struct MotionReference
    {
        double surge{0.0};
        double sway{0.0};
        double heave{0.0};
        double yaw_rate{0.0};
        RcSpeedGear gear{RcSpeedGear::SLOW};
    };

    struct Config
    {
        std::string autostart_module{"pipeline_inspection"};
        std::string observation_topic{"/inspection/pipeline_observation"};

        bool auto_start_inspection{true};

        double command_rate_hz{20.0};
        double observation_timeout_sec{0.5};
        double line_stable_hold_sec{0.5};
        double align_hold_sec{1.0};
        double turn_trigger_hold_sec{0.3};
        double turn_commit_sec{0.5};
        double turn_timeout_sec{4.0};
        double recovery_timeout_sec{3.0};
        double search_reverse_interval_sec{3.0};
        double inspection_duration_sec{0.0};
        double min_observation_confidence{0.6};

        int min_primary_line_points{8};
        int min_secondary_line_points{8};

        double reference_row_ratio{0.65};
        double lateral_tolerance_norm{0.08};
        double angle_tolerance_rad{0.10};

        double sway_from_lateral_gain{0.8};
        double sway_from_lateral_rate_gain{0.08};
        double yaw_rate_from_angle_gain{0.35};
        double yaw_rate_from_lateral_gain{0.10};

        double align_surge_command{0.25};
        double track_surge_command{0.65};
        double turn_surge_command{0.25};
        double search_yaw_command{0.35};
        double recovery_yaw_command{0.30};
        double turn_yaw_rate{0.18};

        double max_forward_speed{0.20};
        double max_lateral_speed{0.15};
        double max_yaw_rate{0.25};

        double error_filter_tau_sec{0.15};
        double error_rate_filter_tau_sec{0.25};
        double minimum_surge_scale{0.15};
        double surge_lateral_error_weight{2.5};
        double surge_angle_error_weight{2.0};
        double hard_stop_lateral_error{0.65};
        double hard_stop_angle_error{0.60};

        double medium_gear_lateral_error{0.18};
        double medium_gear_angle_error{0.20};
        double fast_gear_lateral_error{0.06};
        double fast_gear_angle_error{0.08};
        double fast_gear_min_confidence{0.80};

        double yaw_rate_feedback_kp{0.45};
        double linear_velocity_feedback_kp{0.15};
        double min_linear_velocity_quality{0.60};
        double feedback_timeout_sec{0.30};

        double corner_approach_bottom_distance_px{240.0};
        double turn_trigger_bottom_distance_px{120.0};
        double turn_direction_sign_from_image_x{1.0};

        double module_request_interval_sec{1.0};
        double service_wait_sec{0.2};
    };

    using ControlCommand = VehicleCommandInterface::Command;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    VehicleCommandInterface* vehicle_;

    ros::Subscriber observation_sub_;
    ros::ServiceClient module_control_client_;

    Config config_;
    Observation last_observation_;
    InspectionState state_{InspectionState::BOOTSTRAP};

    bool module_started_{false};
    bool turn_direction_ready_{false};
    double turn_direction_sign_{0.0};
    RecoveryTarget recovery_target_{RecoveryTarget::PRIMARY};
    double recovery_scan_sign_{1.0};

    bool error_filter_initialized_{false};
    TargetLine error_filter_target_{TargetLine::PRIMARY};
    ControlErrors filtered_errors_;

    ros::Time last_module_request_time_;
    ros::Time state_entered_at_;
    ros::Time last_error_update_time_;
    ros::Time line_stable_since_;
    ros::Time align_stable_since_;
    ros::Time turn_ready_since_;
    ros::Time turn_started_at_;
    ros::Time tracking_started_at_;

    void loadParameters();
    void observationCb(
        const scheduler::PipelineCenterlineObservation::ConstPtr& msg);

    void stepStateMachine();
    void enterState(InspectionState next_state, const std::string& reason);
    std::string stateToString(InspectionState state) const;

    void handleStartInspection();
    void handleSearchPrimaryLine();
    void handleAlignPrimaryLine();
    void handleTrackPrimaryLine();
    void handleApproachCorner();
    void handleTurningToSecondaryLine();
    void handleAlignSecondaryLine();
    void handleTrackSecondaryLine();
    void handleRecoverLine();
    void handleFinishing();

    void beginRecovery(RecoveryTarget target, const std::string& reason);

    bool hasFreshObservation() const;
    bool hasUsablePrimaryLine() const;
    bool hasUsableSecondaryLine() const;

    bool tryCallModuleCommand(
        const std::string& module_name,
        const std::string& command,
        scheduler::ControlModuleResponse* response);

    bool fitLinePoints(
        const std::vector<ImagePoint>& points,
        int min_points,
        FittedLine* fitted_line) const;
    bool getTargetLine(TargetLine target, FittedLine* fitted_line) const;

    double getReferenceRowPx() const;
    bool computeTargetErrors(
        TargetLine target,
        double* lateral_error_norm,
        double* angle_error_rad) const;
    bool isTargetAligned(TargetLine target) const;

    bool getIntersectionBottomDistancePx(double* bottom_distance_px) const;
    bool resolveTurnDirection(double* turn_direction_sign) const;

    ControlCommand buildTrackingCommand(
        TargetLine target,
        bool tracking_mode);
    ControlCommand buildTurningCommand();
    ControlCommand buildSearchCommand() const;
    ControlCommand buildRecoveryCommand() const;
    ControlCommand buildRcCommand(const MotionReference& reference) const;

    bool updateControlErrors(TargetLine target, ControlErrors* errors);
    RcSpeedGear selectTrackingGear(
        const ControlErrors& errors,
        bool tracking_mode) const;
    bool getFreshFeedback(VehicleCommandInterface::Feedback* feedback) const;
};
