#pragma once

#include <cmath>

#include <ros/ros.h>

#include <scheduler/mission_controller.h>
#include <scheduler/vehicle_command_interface.h>

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
    ERROR
};

struct DockingData
{
    bool blue_light_detected = false;
    bool dock_pose_valid = false;
    bool apriltag_detected = false;
    bool captured = false;

    double blue_light_yaw_error = 0.0;

    double dock_yaw_error = 0.0;
    double dock_depth_error = 0.0;
    double dock_distance = 0.0;

    double tag_yaw_error = 0.0;
    double tag_depth_error = 0.0;
    double tag_forward_error = 0.0;

    int blue_light_detected_count = 0;
    int tag_detected_count = 0;

    bool new_remote_light = false;
    bool new_dock_pose = false;
    bool new_tag = false;
};

struct SearchConfig
{
    double scan_yaw_rate = 0.1 * M_PI;
    double timeout_s = 40;
    double retry_times = 3;
    double stable_frame = 5;
};

struct ApproachConfig
{
    double forward_speed = 0.20;           
    double yaw_kp = 0.8;                  
};

struct AlignConfig
{
    bool ready_for_tag = false;
    double yaw_deg_tolerance = 5.0;
    double depth_m_tolerance = 0.08;
    double horizon_m_tolerance = 0.07;
    double yaw_kp = 0.6;
    double sway_kp = 0.5;
    double heave_kp = 0.5;
};

struct AlignWithTagConfig
{
    double yaw_deg_tolerance = 2.0;
    double depth_m_tolerance = 0.04;
    double horizon_m_tolerance = 0.03;
    double yaw_kp = 0.4;
    double sway_kp = 0.3;
    double heave_kp = 0.3;
};

struct EnterDockConfig
{
    double forward_speed = 0.08;
    double max_duration = 8.0;
};

struct CapturedConfig
{
    double timeout = 20.0;
};

struct FSMData
{
    State state = State::START;
    ros::Time entertime = 0.0;

    DockingData docking_data;
    SearchConfig search_config;
    ApproachConfig approach_config;
    AlignConfig align_config;
    AlignWithTagConfig align_with_tag_config;
    EnterDockConfig enterdock_config;
    CapturedConfig captured_config;
};

struct DockingConfig
    {
        std::string autostart_module{"docking"};
        std::string observation_topic{""};

        bool auto_start_inspection{true};

        double command_rate_hz{20.0};
        double observation_timeout_sec{0.5};
        double module_request_interval_sec{1.0};
        double service_wait_sec{0.2};
    };
    

class DockingController : public MissionController
{
private:
    FSMData fsm_data;
    DockingConfig docking_config;

    using ControlCommand = VehicleCommandInterface::Command;

    ros::ServiceClient module_control_client_;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    VehicleCommandInterface* vehicle_;

    bool module_started_{false};

    void loadParameters();
    void RemoteBlueLightObservationCb();
    void DockObservationCb();
    void AprilTagObservationCb();
    void EnterState(State next_state);
    void step();
    std::string stateToString(InspectionState state) const;

    void handleStart();
    void handleSearch();
    void handleApproach();
    void handleAlign();
    void handleAlignWithTag();
    void handleEnterDock();
    void handleCaptured();
    void handleCompleted();
    void handleError();

    bool tryCallModuleCommand(
        const std::string& module_name,
        const std::string& command,
        scheduler::ControlModuleResponse* response);

    ControlCommand UpdateApproachCmd();
    ControlCommand UpdateAlignCmd();
    ControlCommand UpdateEnterDockCmd();
    
public:
    DockingController(
        ros::NodeHandle nh,
        ros::NodeHandle pnh,
        VehicleCommandInterface* vehicle);

    void run() override;
    std::string missionType() const override;
};

