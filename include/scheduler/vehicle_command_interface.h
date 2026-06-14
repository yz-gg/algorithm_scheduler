#pragma once

#include <string>

#include <ros/ros.h>

#include <scheduler/rc_command_mapper.h>

class VehicleCommandInterface
{
public:
    enum class CommandType
    {
        HOLD,
        BODY_VELOCITY,
        LOCAL_POSE,
        RC_OVERRIDE,
        RC_RELEASE,
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
        RcNormalizedCommand rc_override;
    };

    struct Feedback
    {
        bool state_valid{false};
        bool connected{false};
        bool armed{false};
        std::string mode;

        bool angular_velocity_valid{false};
        bool linear_acceleration_valid{false};
        bool linear_velocity_valid{false};
        bool attitude_valid{false};
        bool local_position_valid{false};

        double roll_rate{0.0};
        double pitch_rate{0.0};
        double yaw_rate{0.0};

        double body_ax{0.0};
        double body_ay{0.0};
        double body_az{0.0};

        double body_vx{0.0};
        double body_vy{0.0};
        double body_vz{0.0};
        double linear_velocity_quality{0.0};

        double roll{0.0};
        double pitch{0.0};
        double yaw{0.0};
        double local_x{0.0};
        double local_y{0.0};
        double local_z{0.0};

        ros::Time state_stamp;
        ros::Time angular_velocity_stamp;
        ros::Time linear_acceleration_stamp;
        ros::Time local_pose_stamp;
        ros::Time stamp;
    };

    virtual ~VehicleCommandInterface() = default;

    virtual bool waitForConnection() = 0;
    virtual bool waitForServices(const ros::Duration& timeout) = 0;
    virtual bool setMode(const std::string& mode_name) = 0;
    virtual bool arm(bool value) = 0;
    virtual bool isArmed() const = 0;
    virtual bool getFeedback(Feedback* feedback) const
    {
        (void)feedback;
        return false;
    }

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

    static Command RcOverride(
        double surge,
        double sway,
        double heave,
        double yaw,
        RcSpeedGear gear)
    {
        Command command;
        command.type = CommandType::RC_OVERRIDE;
        command.rc_override.surge = surge;
        command.rc_override.sway = sway;
        command.rc_override.heave = heave;
        command.rc_override.yaw = yaw;
        command.rc_override.gear = gear;
        return command;
    }

    static Command RcNeutral()
    {
        return RcOverride(0.0, 0.0, 0.0, 0.0, RcSpeedGear::SLOW);
    }

    static Command RcRelease()
    {
        Command command;
        command.type = CommandType::RC_RELEASE;
        return command;
    }
};
