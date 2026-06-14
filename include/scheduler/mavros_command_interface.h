#pragma once

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <scheduler/vehicle_command_interface.h>

class MavrosCommandInterface : public VehicleCommandInterface
{
public:
    MavrosCommandInterface(ros::NodeHandle nh, ros::NodeHandle pnh);

    bool waitForConnection() override;
    bool waitForServices(const ros::Duration& timeout) override;
    bool setMode(const std::string& mode_name) override;
    bool setCustomMode(uint32_t custom_mode_id);
    bool arm(bool value) override;
    bool isArmed() const override;
    bool getFeedback(Feedback* feedback) const override;

    void publishCommand(const Command& command) override;
    void publishHold() override;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber state_sub_;
    ros::Subscriber imu_sub_;
    ros::Subscriber local_pose_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher local_pose_pub_;
    ros::Publisher rc_override_pub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::State current_state_;
    Feedback feedback_;

    RcCommandMapper rc_mapper_;
    RcPwmCommand last_rc_pwm_;
    int rc_pwm_slew_step_{20};

    void stateCb(const mavros_msgs::State::ConstPtr& msg);
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg);
    void localPoseCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void publishBodyVelocity(const BodyVelocityCommand& command);
    void publishLocalPose(const LocalPoseCommand& command);
    void publishRcOverride(const RcNormalizedCommand& command);
    void releaseRcOverride();
    RcPwmCommand applyRcSlewLimit(const RcPwmCommand& target) const;
    bool transmitRcPwm(const RcPwmCommand& command);
};
