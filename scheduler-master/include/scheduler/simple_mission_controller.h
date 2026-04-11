#pragma once

#include <string>

#include <ros/ros.h>

#include <scheduler/mission_controller.h>
#include <scheduler/vehicle_command_interface.h>

class SimpleMissionController : public MissionController
{
public:
    SimpleMissionController(
        ros::NodeHandle nh,
        ros::NodeHandle pnh,
        VehicleCommandInterface* vehicle,
        std::string mission_type,
        std::string default_module_name);

    void run() override;
    std::string missionType() const override;

private:
    struct Config
    {
        std::string autostart_module;
        bool auto_start_module{true};
        double command_rate_hz{10.0};
        double module_request_interval_sec{1.0};
        double service_wait_sec{0.2};

        std::string command_type{"hold"};

        double body_vx{0.0};
        double body_vy{0.0};
        double body_vz{0.0};
        double body_yaw_rate{0.0};

        double local_pose_x{0.0};
        double local_pose_y{0.0};
        double local_pose_z{0.0};
        double local_pose_yaw{0.0};
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    VehicleCommandInterface* vehicle_;
    ros::ServiceClient module_control_client_;

    std::string mission_type_;
    Config config_;
    bool module_started_{false};
    ros::Time last_module_request_time_;

    void loadParameters(const std::string& default_module_name);
    bool tryCallModuleCommand(const std::string& module_name, const std::string& command);
    bool maybeStartModule();
    void maybeStopModule();
    VehicleCommandInterface::Command buildCommand() const;
};
