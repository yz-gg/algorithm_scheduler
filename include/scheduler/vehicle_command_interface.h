#pragma once

#include <string>

#include <ros/ros.h>

class VehicleCommandInterface
{
public:
    enum class CommandType
    {
        HOLD,
        BODY_VELOCITY,
        LOCAL_POSE,
    };

    struct BodyVelocityCommand
    {
        double vx{0.0};
        double vy{0.0};
        double vz{0.0};
        double roll_rate{0.0};
        double pitch_rate{0.0};
        double yaw_rate{0.0};
    };

    struct LocalPoseCommand
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double yaw{0.0};
    };

    struct Command
    {
        CommandType type{CommandType::HOLD};
        BodyVelocityCommand body_velocity;
        LocalPoseCommand local_pose;
    };

    virtual ~VehicleCommandInterface() = default;

    virtual bool waitForConnection() = 0;
    virtual bool waitForServices(const ros::Duration& timeout) = 0;
    virtual bool setMode(const std::string& mode_name) = 0;
    virtual bool arm(bool value) = 0;

    virtual void publishCommand(const Command& command) = 0;
    virtual void publishHold() = 0;

    static Command Hold()
    {
        return Command{};
    }

    static Command BodyVelocity(
        double vx,
        double vy,
        double vz,
        double yaw_rate)
    {
        Command command;
        command.type = CommandType::BODY_VELOCITY;
        command.body_velocity.vx = vx;
        command.body_velocity.vy = vy;
        command.body_velocity.vz = vz;
        command.body_velocity.yaw_rate = yaw_rate;
        return command;
    }

    static Command LocalPose(
        double x,
        double y,
        double z,
        double yaw)
    {
        Command command;
        command.type = CommandType::LOCAL_POSE;
        command.local_pose.x = x;
        command.local_pose.y = y;
        command.local_pose.z = z;
        command.local_pose.yaw = yaw;
        return command;
    }
};
