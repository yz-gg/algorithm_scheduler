#pragma once

#include <string>

class MissionController
{
public:
    virtual ~MissionController() = default;

    virtual void run() = 0;
    virtual std::string missionType() const = 0;
};
