#pragma once

#include <memory>
#include <string>

#include <ros/ros.h>

#include <scheduler/mission_controller.h>
#include <scheduler/vehicle_command_interface.h>

std::unique_ptr<MissionController> CreateMissionController(
    const std::string& mission_type,
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    VehicleCommandInterface* vehicle);
