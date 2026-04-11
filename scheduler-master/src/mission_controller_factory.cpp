#include <scheduler/mission_controller_factory.h>

#include <memory>
#include <utility>

#include <scheduler/pipeline_inspection_controller.h>
#include <scheduler/simple_mission_controller.h>

std::unique_ptr<MissionController> CreateMissionController(
    const std::string& mission_type,
    ros::NodeHandle nh,
    ros::NodeHandle pnh,
    VehicleCommandInterface* vehicle)
{
    if (mission_type == "pipeline" || mission_type == "pipeline_inspection")
    {
        return std::unique_ptr<MissionController>(
            new PipelineInspectionController(std::move(nh), std::move(pnh), vehicle));
    }

    if (mission_type == "cage" || mission_type == "cage_inspection")
    {
        return std::unique_ptr<MissionController>(
            new SimpleMissionController(
                std::move(nh),
                std::move(pnh),
                vehicle,
                "cage_inspection",
                "cage_inspection"));
    }

    if (mission_type == "dam" || mission_type == "dam_inspection")
    {
        return std::unique_ptr<MissionController>(
            new SimpleMissionController(
                std::move(nh),
                std::move(pnh),
                vehicle,
                "dam_inspection",
                "dam_inspection"));
    }

    if (mission_type == "dock" || mission_type == "docking")
    {
        return std::unique_ptr<MissionController>(
            new SimpleMissionController(
                std::move(nh),
                std::move(pnh),
                vehicle,
                "docking",
                "docking"));
    }

    return std::unique_ptr<MissionController>();
}
