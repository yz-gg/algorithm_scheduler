#include <scheduler/docking_controller.h>

#include <cmath>

void DockingController::step()
{

}


DockingController::DockingController(
        ros::NodeHandle nh,
        ros::NodeHandle pnh,
        VehicleCommandInterface* vehicle) :
        nh_(nh), pnh_(pnh), vehicle_(vehicle)
        {
            loadParameters();
        }

void DockingController::loadParameters()
{

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

}

std::string DockingController::stateToString(State state) const
{

}

void DockingController::handleStart()
{

}

void DockingController::handleSearch()
{

}

void DockingController::handleApproach()
{

}

void DockingController::handleAlign()
{

}

void DockingController::handleAlignWithTag()
{

}

void DockingController::handleEnterDock()
{

}

void DockingController::handleCaptured()
{

}

void DockingController::handleCompleted()
{

}

void DockingController::handleError();