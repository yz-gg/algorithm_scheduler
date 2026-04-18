#include <scheduler/docking_controller.h>

#include <cmath>

/* 
// TODO
// loadParaments->codex
// 算法接入，需要知道订阅什么话题
// ->根据接入的算法，写回调函数
// ->写handleStart

*/
namespace
{
double clampValue(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}
}

void DockingController::run()
{
    ros::Rate rate(fsm_data.docking_data.command_rate_hz);

    ROS_INFO("Docking controller started");

    if (fsm_data.docking_data.auto_start_inspection)
    {
        enterState(State::START);
    }
    else
    {
        enterState(State::SEARCH_BLUE_LIGHT);
    }

    while (ros::ok())
    {
        ros::spinOnce();
        step();
        rate.sleep();
    }
}

void DockingController::step()
{
    switch (fsm_data.state)
    {
    case START:
        handleStart();
        break;
    case SEARCH_BLUE_LIGHT:
        handleSearch();
        break;
    case APPROACH_BLUE_LIGHT:
        handleApproach();
        break;
    case ALIGN_WITH_DOCK:
        handleAlign();
        break;
    case ALIGN_WITH_APRILTAG:
        handleAlignWithTag();
        break;
    case ENTER_DOCK:
        handleEnterDock();
        break;
    case WAIT_CAPTURE:
        handleCaptured();
        break;
    case COMPLETED:
        handleCompleted();
        break;
    case ERROR:
        handleError();
        break;  
    default:
        break;
    }
}

std::string DockingController::missionType() const
{
    return "Docking";
}


DockingController::DockingController(
        ros::NodeHandle nh,
        ros::NodeHandle pnh,
        VehicleCommandInterface* vehicle) :
        nh_(std::move(nh)), pnh_(std::move(pnh)), vehicle_(vehicle)
        {
            loadParameters();

            
            module_control_client_ = nh_.serviceClient<scheduler::ControlModule>(
            "/launch_manager_node/control_module");
        }

void DockingController::loadParameters()
{
    pnh_.param<std::string>(
        "autostart_module", docking_config.autostart_module, docking_config.autostart_module
    )
    pnh_.param<std::string>(
        "observation_topic", docking_config.observation_topic, docking_config.observation_topic
    )
    pnh_.param(
        "auto_start_inspection", docking_config.auto_start_inspection, docking_config.auto_start_inspection
    )
    pnh_.param(
        "command_rate_hz", docking_config.command_rate_hz, docking_config.command_rate_hz
    )
    pnh_.param(
        "observation_timeout_sec", docking_config.observation_timeout_sec, docking_config.observation_timeout_sec
    )
    pnh_.param(
        "module_request_interval_sec", docking_config.module_request_interval_sec, docking_config.module_request_interval_sec
    )
    pnh_.param(
        "service_wait_sec", docking_config.service_wait_sec, docking_config.service_wait_sec
    )
    pnh_.param(
        "scan_yaw_rate", fsm_data.search_config.scan_yaw_rate, fsm_data.search_config.scan_yaw_rate
    )
    

}

void DockingController::RemoteBlueLightObservationCb()
{

}

void DockingController::DockObservationCb()
{

}

void DockingController::AprilTagObservationCb()
{

}

void DockingController::EnterState(State next_state)
{
    fsm_data.state = next_state;
    fsm_data.entertime = ros::Time();
}

std::string DockingController::stateToString(State state) const
{
    switch (state)
    {
        std::string str = "";
    case START:
        str = "START";
        break;
    case SEARCH_BLUE_LIGHT:
        str = "SEARCH_BLUE_LIGHT";
        break;
        case APPROACH_BLUE_LIGHT:
        str = "APPROACH_BLUE_LIGHT";
        break;
        case ALIGN_WITH_DOCK:
        str = "ALIGN_WITH_DOCK";
        break;
        case ALIGN_WITH_APRILTAG:
        str = "ALIGN_WITH_APRILTAG";
        break;
        case ENTER_DOCK:
        str = "ENTER_DOCK";
        break;
        case WAIT_CAPTURE:
        str = "WAIT_CAPTURE";
        break;
        case COMPLETED:
        str = "COMPLETED";
        break;
        case ERROR:
        str = "ERROR";
        break;
    default:
        break;
        return str;
    }
}

/* 
    srv->start algorithm module
    if suc->search?
    timeout?
*/
void DockingController::handleStart()
{

    EnterState(SEARCH_BLUE_LIGHT);
}

/* 
    超时?
*/
void DockingController::handleSearch()
{
    yaw = fsm_data.search_config.scan_yaw_rate;

     vehicle_->publishCommand(BulidCmd(fsm_data.state));

    if (fsm_data.docking_data.blue_light_detected && fsm_data.docking_data.new_remote_light)
    {
        fsm_data.docking_data.blue_light_detected_count++;

        if (fsm_data.docking_data.blue_light_detected_count > fsm_data.search_config.stable_frame)
        {
        EnterState(APPROACH_BLUE_LIGHT);
        fsm_data.docking_data.Reset();
        }

        fsm_data.docking_data.new_remote_light = false;
    }
    else if (!fsm_data.docking_data.blue_light_detected)
    {
        fsm_data.docking_data.blue_light_detected_count = 0;
    }

}

void DockingController::handleApproach()
{
    if (fsm_data.docking_data.blue_light_detected && fsm_data.docking_data.new_remote_light)
    {
        fsm_data.docking_data.yaw = fsm_data.approach_config.yaw_kp * fsm_data.docking_data.blue_light_yaw_error;
        fsm_data.docking_data.surge = fsm_data.approach_config.forward_speed;

        vehicle_->publishCommand(BulidCmd(fsm_data.state));

        fsm_data.docking_data.new_remote_light = false;
    }
    else
    {
        vehicle_->publishHold();
    }
    

    if (fsm_data.docking_data.dock_pose_valid && fsm_data.docking_data.new_remote_light)
    {
        fsm_data.docking_data.dock_detected_count++;
        fsm_data.docking_data.new_dock_pose = false;

        if (fsm_data.docking_data.dock_detected_count > fsm_data.approach_config.stable_frame)
        {
            EnterState(ALIGN_WITH_DOCK);
            fsm_data.docking_data.Reset();
        }
    }
    else if (!fsm_data.docking_data.dock_pose_valid)
    {
        fsm_data.docking_data.dock_detected_count=0;
    }

}

void DockingController::handleAlign()
{
    if (fsm_data.docking_data.dock_pose_valid && fsm_data.docking_data.new_dock_pose)
    {
        fsm_data.docking_data.yaw = fsm_data.docking_data.dock_yaw_error * fsm_data.align_config.yaw_kp;
        fsm_data.docking_data.sway = fsm_data.docking_data.dock_sway_error * fsm_data.align_config.sway_kp;
        fsm_data.docking_data.heave = fsm_data.docking_data.dock_depth_error * fsm_data.align_config.heave_kp;
        if (fsm_data.docking_data.dock_distance > fsm_data.align_config.dist_tolerance)
        {
            fsm_data.docking_data.surge = fsm_data.align_config.surge_speed;
        }
        else
        {
            fsm_data.docking_data.surge = 0;
        }

        vehicle_->publishCommand(BulidCmd(fsm_data.state));
        fsm_data.docking_data.new_dock_pose = false;
    }
    else
    {
        vehicle_->publishHold();
    }
    
    if (ReadyForTag() && fsm_data.docking_data.apriltag_detected && fsm_data.docking.new_tag)
    {
        fsm_data.docking_data.tag_detected_count++;

        fsm_data.docking.new_tag = false;

        if (fsm_data.docking_data.tag_detected_count > fsm_data.align_config.stable_frame)
        {
            EnterState(ALIGN_WITH_APRILTAG);
            fsm_data.docking_data.Reset();
        }
    }
    else if (!ReadyForTag() || !fsm_data.docking_data.apriltag_detected)
    {
        fsm_data.docking_data.tag_detected_count = 0;
    }
}

void DockingController::handleAlignWithTag()
{
    if (fsm_data.docking_data.apriltag_detected && fsm_data.docking_data.new_tag)
    {
        fsm_data.docking_data.surge = 0;
        fsm_data.docking_data.sway = fsm_data.align_with_tag_config.sway_kp * fsm_data.docking_data.tag_sway_error;
        fsm_data.docking_data.heave = fsm_data.align_with_tag_config.heave_kp * fsm_data.docking_data.tag_depth_error;
        fsm_data.docking_data.yaw = fsm_data.align_with_tag_config.yaw_kp * fsm_data.docking_data.tag_yaw_error;

        vehicle_->publishCommand(BulidCmd(fsm_data.state));
        fsm_data.docking_data.new_tag = false;
    }
    else
    {
        vehicle_->publishHold();
    }

    if (ReadyForEnterDock() && fsm_data.docking_data.new_tag)
    {
        fsm_data.docking_data.tag_stable_count++;
        fsm_data.docking_data.new_tag = 0;

        if (fsm_data.docking_data.tag_stable_count > fsm_data.align_with_tag_config.stable_frame)
        {
            EnterState(ENTER_DOCK);
            docking_data.Reset();
        }
    }
    else if (!ReadyForEnterDock())
    {
        fsm_data.docking_data.tag_stable_count = 0;
    }
}

void DockingController::handleEnterDock()
{
    fsm_data.docking_data.surge = fsm_data.enterdock_config.forward_speed;
     vehicle_->publishCommand(BulidCmd(fsm_data.state));
}

void DockingController::handleCaptured()
{

}

void DockingController::handleCompleted()
{

}

void DockingController::handleError()
{
    
}

void DockingData::Reset()
{
    *this = {};
}

bool DockingController::ReadyForTag()
{
    return fsm_data.docking_data.dock_yaw_error < fsm_data.align_config.yaw_deg_tolerance &&
            fsm_data.docking_data.dock_depth_error < fsm_data.align_config.depth_m_tolerance &&
            fsm_data.docking_data.dock_sway_error < fsm_data.align_config.sway_m_tolerance &&
            fsm_data.docking_data.dock_distance < fsm_data.align_config.dist_tolerance
} 

bool DockingController::ReadyForEnterDock()
{
    return fsm_data.docking_data.tag_yaw_error  < fsm_data.align_with_tag_config.yaw_deg_tolerance &&
            fsm_data.docking_data.tag_depth_error  < fsm_data.align_with_tag_config.depth_m_tolerance &&
            fsm_data.docking_data.tag_sway_error  < fsm_data.align_with_tag_config.sway_m_tolerance
}

ControlCommand DockingController::BulidCmd(State s)
{
    switch (s)
    {
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;
        double yaw  = 0.0;
    case SEARCH_BLUE_LIGHT:
        vx = clampValue(fsm_data.docking_data.surge, -fsm_data.search_config.max_surge,fsm_data.search_config.max_surge);
        vy = clampValue(fsm_data.docking_data.sway, -fsm_data.search_config.max_sway,fsm_data.search_config.max_sway);
        vz = clampValue(fsm_data.docking_data.heave, -fsm_data.search_config.max_heave,fsm_data.search_config.max_heave);
        yaw = clampValue(fsm_data.docking_data.yaw, -fsm_data.search_config.max_yaw,fsm_data.search_config.max_yaw);
        break;
    case APPROACH_BLUE_LIGHT:
        vx = clampValue(fsm_data.docking_data.surge, -fsm_data.approach_config.max_surge,fsm_data.approach_config.max_surge);
        vy = clampValue(fsm_data.docking_data.sway, -fsm_data.approach_config.max_sway,fsm_data.approach_config.max_sway);
        vz = clampValue(fsm_data.docking_data.heave, -fsm_data.approach_config.max_heave,fsm_data.approach_config.max_heave);
        yaw = clampValue(fsm_data.docking_data.yaw, -fsm_data.approach_config.max_yaw,fsm_data.approach_config.max_yaw);
        break;
    case ALIGN_WITH_DOCK:
        vx = clampValue(fsm_data.docking_data.surge, -fsm_data.align_config.max_surge,fsm_data.align_config.max_surge);
        vy = clampValue(fsm_data.docking_data.sway, -fsm_data.align_config.max_sway,fsm_data.align_config.max_sway);
        vz = clampValue(fsm_data.docking_data.heave, -fsm_data.align_config.max_heave,fsm_data.align_config.max_heave);
        yaw = clampValue(fsm_data.docking_data.yaw, -fsm_data.align_config.max_yaw,fsm_data.align_config.max_yaw);
        break;
    case ALIGN_WITH_APRILTAG:
        vx = clampValue(fsm_data.docking_data.surge, -fsm_data.align_with_tag_config.max_surge,fsm_data.align_with_tag_config.max_surge);
        vy = clampValue(fsm_data.docking_data.sway, -fsm_data.align_with_tag_config.max_sway,fsm_data.align_with_tag_config.max_sway);
        vz = clampValue(fsm_data.docking_data.heave, -fsm_data.align_with_tag_config.max_heave,fsm_data.align_with_tag_config.max_heave);
        yaw = clampValue(fsm_data.docking_data.yaw, -fsm_data.align_with_tag_config.max_yaw,fsm_data.align_with_tag_config.max_yaw);
        break;
    case ENTER_DOCK:
        vx = clampValue(fsm_data.docking_data.surge, -fsm_data.enterdock_config.max_surge,fsm_data.enterdock_config.max_surge);
        vy = clampValue(fsm_data.docking_data.sway, -fsm_data.enterdock_config.max_sway,fsm_data.enterdock_config.max_sway);
        vz = clampValue(fsm_data.docking_data.heave, -fsm_data.enterdock_config.max_heave,fsm_data.enterdock_config.max_heave);
        yaw = clampValue(fsm_data.docking_data.yaw, -fsm_data.enterdock_config.max_yaw,fsm_data.enterdock_config.max_yaw);
        break;
    default:
        break;
    }
    

    return VehicleCommandInterface::BodyVelocity(vx, vy, vz, yaw);
}