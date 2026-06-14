#include <ros/ros.h>

#include <scheduler/mavros_command_interface.h>
#include <scheduler/mission_controller_factory.h>

#include <memory>
#include <string>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scheduler_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    bool wait_for_fcu_connection = true;
    bool set_flight_mode_on_start = true;
    bool auto_arm = true;
    std::string flight_mode = "GUIDED";
    std::string mission_type = "pipeline_inspection";
    double mavros_service_wait_sec = 5.0;

    pnh.param(
        "wait_for_fcu_connection",
        wait_for_fcu_connection,
        wait_for_fcu_connection);
    pnh.param(
        "set_flight_mode_on_start",
        set_flight_mode_on_start,
        set_flight_mode_on_start);
    pnh.param("auto_arm", auto_arm, auto_arm);
    pnh.param<std::string>("flight_mode", flight_mode, flight_mode);
    pnh.param<std::string>("mission_type", mission_type, mission_type);
    pnh.param(
        "mavros_service_wait_sec",
        mavros_service_wait_sec,
        mavros_service_wait_sec);

    MavrosCommandInterface mavros(nh, pnh);

    ROS_INFO_STREAM(
        "Scheduler node started with mission type [" << mission_type << "]");

    if (wait_for_fcu_connection && !mavros.waitForConnection())
    {
        ROS_ERROR("FCU connection was not established");
        return 1;
    }

    if (!mavros.waitForServices(ros::Duration(mavros_service_wait_sec)))
    {
        ROS_WARN("One or more MAVROS services are not available yet");
    }

    bool mode_set = true;
    if (set_flight_mode_on_start)
    {
        mode_set = mavros.setMode(flight_mode);
        if (!mode_set)
        {
            ROS_WARN("Failed to set flight mode to %s", flight_mode.c_str());
        }
    }

    if (!mode_set)
    {
        ROS_WARN("Vehicle mode switch request was rejected");
    }

    if (auto_arm && !mavros.arm(true))
    {
        ROS_WARN("Failed to arm vehicle");
    }

    std::unique_ptr<MissionController> controller =
        CreateMissionController(mission_type, nh, pnh, &mavros);
    if (!controller)
    {
        ROS_ERROR("Unsupported mission type: %s", mission_type.c_str());
        return 1;
    }

    controller->run();
    return 0;
}
