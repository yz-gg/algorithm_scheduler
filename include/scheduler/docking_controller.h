#pragma once

#include <cmath>
#include <string>

#include <ros/ros.h>

#include <scheduler/ControlModule.h>
#include <scheduler/mission_controller.h>
#include <scheduler/vehicle_command_interface.h>

class DockingController : public MissionController
{
public:
    DockingController(
        ros::NodeHandle nh,
        ros::NodeHandle pnh,
        VehicleCommandInterface* vehicle);

    void run() override;
    std::string missionType() const override;

private:
    enum class State
    {
        START,
        SEARCH_BLUE_LIGHT,
        APPROACH_BLUE_LIGHT,
        ALIGN_WITH_DOCK,
        ALIGN_WITH_APRILTAG,
        ENTER_DOCK,
        WAIT_CAPTURE,
        COMPLETED,
        ERROR,
    };

    struct MotionLimitConfig
    {
        MotionLimitConfig(
            double surge_value = 0.0,
            double sway_value = 0.0,
            double heave_value = 0.0,
            double yaw_value = 0.0)
            : surge(surge_value)
            , sway(sway_value)
            , heave(heave_value)
            , yaw(yaw_value)
        {
        }

        double surge{0.0};
        double sway{0.0};
        double heave{0.0};
        double yaw{0.0};
    };

    struct SearchConfig
    {
        double scan_yaw_rate{0.1 * M_PI};
        double timeout_sec{40.0};
        int retry_times{3};
        int stable_frames{5};
        MotionLimitConfig command_limits{0.0, 0.0, 0.0, 0.2};
    };

    struct ApproachConfig
    {
        double forward_speed{0.20};
        double yaw_kp{0.8};
        double heave_kp{0.1};
        int stable_frames{3};
        int lost_frames_tolerance{10};
        MotionLimitConfig command_limits{0.20, 0.1, 0.1, 0.3};

        ros::Time fresh_tolerance_s{5.0};
    };

    struct AlignConfig
    {
        double yaw_deg_tolerance{5.0};
        double depth_m_tolerance{0.08};
        double sway_m_tolerance{0.07};
        double dist_tolerance{0.3};

        double yaw_kp{0.6};
        double sway_kp{0.5};
        double heave_kp{0.5};
        double surge_speed{0.1};

        int stable_frames{7};
        int lost_frames_tolerance{5};
        MotionLimitConfig command_limits{0.1, 0.2, 0.1, 0.2};

        ros::Time fresh_tolerance_s{2.0};
    };

    struct AlignWithTagConfig
    {
        double yaw_deg_tolerance{2.0};
        double depth_m_tolerance{0.04};
        double sway_m_tolerance{0.03};

        double yaw_kp{0.4};
        double sway_kp{0.3};
        double heave_kp{0.3};

        int stable_frames{10};
        int lost_frames_tolerance{3};
        MotionLimitConfig command_limits{0.0, 1.0, 0.05, 0.1};

        ros::Time fresh_tolerance_s{1.0};
    };

    struct EnterDockConfig
    {
        double forward_speed{0.08};
        double max_duration_sec{8.0};
        MotionLimitConfig command_limits{0.08, 0.0, 0.0, 0.0};
    };

    struct CaptureConfig
    {
        double timeout_sec{20.0};
    };

    struct Config
    {
        std::string autostart_module{"docking"};

        std::string blue_light_topic{""};
        std::string dock_pose_topic{""};
        std::string apriltag_topic{""};

        bool auto_start_inspection{true};

        double command_rate_hz{20.0};
        double observation_timeout_sec{0.5};
        double module_request_interval_sec{1.0};
        double service_wait_sec{0.2};

        SearchConfig search;
        ApproachConfig approach;
        AlignConfig align;
        AlignWithTagConfig align_with_tag;
        EnterDockConfig enter_dock;
        CaptureConfig capture;
    };

    struct RuntimeData
    {
        double surge{0.0};
        double sway{0.0};
        double heave{0.0};
        double yaw{0.0};

        bool blue_light_detected{false};
        bool dock_pose_valid{false};
        bool apriltag_detected{false};
        bool captured{false};

        double blue_light_yaw_error{0.0};

        double dock_yaw_error{0.0};
        double dock_depth_error{0.0};
        double dock_sway_error{0.0};
        double dock_distance{0.0};

        double tag_sway_error{0.0};
        double tag_yaw_error{0.0};
        double tag_depth_error{0.0};

        int blue_light_detected_count{0};
        int dock_detected_count{0};
        int tag_detected_count{0};
        int tag_stable_count{0};

        int light_lost_count{0};
        int dock_pose_lost_count{0};
        int tag_lost_count{0};

        bool new_remote_light{false};
        bool new_dock_pose{false};
        bool new_tag{false};

        ros::Time light_timestamp{0.0};
        ros::Time dock_pose_timestamp{0.0};
        ros::Time apriltag_timestamp{0.0};

        void reset();
        void clearCommand();
    };

    using ControlCommand = VehicleCommandInterface::Command;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    VehicleCommandInterface* vehicle_;

    ros::ServiceClient module_control_client_;

    Config config_;
    RuntimeData data_;
    State state_{State::START};

    ros::Time state_entered_at_;
    ros::Time last_module_request_time_;

    bool module_started_{false};
    int search_retry_count_{0};

    void loadParameters();
    void loadMotionLimits(const std::string& prefix, MotionLimitConfig* limits);

    void step();
    void enterState(State next_state, const std::string& reason);
    std::string stateToString(State state) const;
    double stateElapsedSec() const;

    void handleStart();
    void handleSearch();
    void handleApproach();
    void handleAlign();
    void handleAlignWithTag();
    void handleEnterDock();
    void handleCaptured();
    void handleCompleted();
    void handleError();

    void BlueLightCB();
    void DockPoseCB();
    void AprilTagCB();

    void checkObversationFresh();

    bool readyForTag() const;
    bool readyForEnterDock() const;

    bool tryCallModuleCommand(
        const std::string& module_name,
        const std::string& command,
        scheduler::ControlModuleResponse* response);
    bool stopModuleIfNeeded();

    ControlCommand buildCommand(State state) const;
};
