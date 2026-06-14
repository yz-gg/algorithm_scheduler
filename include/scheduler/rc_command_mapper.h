#pragma once

#include <array>
#include <cstdint>
#include <string>

enum class RcSpeedGear
{
    SLOW,
    MEDIUM,
    FAST,
};

struct RcNormalizedCommand
{
    double surge{0.0};
    double sway{0.0};
    double heave{0.0};
    double yaw{0.0};
    RcSpeedGear gear{RcSpeedGear::SLOW};
};

struct RcPwmCommand
{
    std::uint16_t surge{1500};
    std::uint16_t sway{1500};
    std::uint16_t heave{1500};
    std::uint16_t yaw{1500};
};

using RcOverrideChannels = std::array<std::uint16_t, 18>;

struct RcCommandMapperConfig
{
    int center_pwm{1500};
    int slow_max_pwm{1650};
    int medium_max_pwm{1800};
    int fast_max_pwm{1900};

    int surge_direction{1};
    int sway_direction{1};
    int heave_direction{1};
    int yaw_direction{1};

    double input_deadband{0.02};
};

class RcCommandMapper
{
public:
    explicit RcCommandMapper(
        RcCommandMapperConfig config = RcCommandMapperConfig{});

    RcPwmCommand map(const RcNormalizedCommand& command) const;
    RcPwmCommand neutral() const;
    int maxPwm(RcSpeedGear gear) const;

private:
    RcCommandMapperConfig config_;

    std::uint16_t mapAxis(
        double normalized_value,
        RcSpeedGear gear,
        int direction) const;
};

const char* RcSpeedGearToString(RcSpeedGear gear);
RcSpeedGear ParseRcSpeedGear(
    const std::string& value,
    RcSpeedGear fallback = RcSpeedGear::SLOW);

RcOverrideChannels BuildRcOverrideChannels(
    const RcPwmCommand& command,
    std::uint16_t center_pwm = 1500);
RcOverrideChannels BuildRcReleaseChannels();
