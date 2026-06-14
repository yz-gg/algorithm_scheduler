#include <scheduler/rc_command_mapper.h>

#include <algorithm>
#include <cmath>

namespace
{
constexpr std::uint16_t kRcRelease = 0;
constexpr std::uint16_t kRcNoChange = 65535;

double clampValue(double value, double min_value, double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

int normalizedDirection(int direction)
{
    return direction < 0 ? -1 : 1;
}
}

RcCommandMapper::RcCommandMapper(RcCommandMapperConfig config)
    : config_(config)
{
    config_.slow_max_pwm = std::max(config_.slow_max_pwm, config_.center_pwm);
    config_.medium_max_pwm =
        std::max(config_.medium_max_pwm, config_.slow_max_pwm);
    config_.fast_max_pwm =
        std::max(config_.fast_max_pwm, config_.medium_max_pwm);
    config_.input_deadband = clampValue(config_.input_deadband, 0.0, 0.5);

    config_.surge_direction = normalizedDirection(config_.surge_direction);
    config_.sway_direction = normalizedDirection(config_.sway_direction);
    config_.heave_direction = normalizedDirection(config_.heave_direction);
    config_.yaw_direction = normalizedDirection(config_.yaw_direction);
}

RcPwmCommand RcCommandMapper::map(const RcNormalizedCommand& command) const
{
    RcPwmCommand pwm;
    pwm.surge = mapAxis(command.surge, command.gear, config_.surge_direction);
    pwm.sway = mapAxis(command.sway, command.gear, config_.sway_direction);
    pwm.heave = mapAxis(command.heave, command.gear, config_.heave_direction);
    pwm.yaw = mapAxis(command.yaw, command.gear, config_.yaw_direction);
    return pwm;
}

RcPwmCommand RcCommandMapper::neutral() const
{
    RcPwmCommand command;
    command.surge = static_cast<std::uint16_t>(config_.center_pwm);
    command.sway = static_cast<std::uint16_t>(config_.center_pwm);
    command.heave = static_cast<std::uint16_t>(config_.center_pwm);
    command.yaw = static_cast<std::uint16_t>(config_.center_pwm);
    return command;
}

int RcCommandMapper::maxPwm(RcSpeedGear gear) const
{
    switch (gear)
    {
    case RcSpeedGear::FAST:
        return config_.fast_max_pwm;
    case RcSpeedGear::MEDIUM:
        return config_.medium_max_pwm;
    case RcSpeedGear::SLOW:
    default:
        return config_.slow_max_pwm;
    }
}

std::uint16_t RcCommandMapper::mapAxis(
    double normalized_value,
    RcSpeedGear gear,
    int direction) const
{
    double value = clampValue(normalized_value, -1.0, 1.0);
    if (std::fabs(value) <= config_.input_deadband)
    {
        value = 0.0;
    }

    value *= static_cast<double>(direction);
    const int pwm_range = maxPwm(gear) - config_.center_pwm;
    const int pwm = config_.center_pwm +
        static_cast<int>(std::lround(value * static_cast<double>(pwm_range)));
    const int min_pwm = config_.center_pwm - pwm_range;
    return static_cast<std::uint16_t>(
        std::max(min_pwm, std::min(pwm, config_.center_pwm + pwm_range)));
}

const char* RcSpeedGearToString(RcSpeedGear gear)
{
    switch (gear)
    {
    case RcSpeedGear::FAST:
        return "fast";
    case RcSpeedGear::MEDIUM:
        return "medium";
    case RcSpeedGear::SLOW:
    default:
        return "slow";
    }
}

RcSpeedGear ParseRcSpeedGear(const std::string& value, RcSpeedGear fallback)
{
    if (value == "slow")
    {
        return RcSpeedGear::SLOW;
    }
    if (value == "medium")
    {
        return RcSpeedGear::MEDIUM;
    }
    if (value == "fast")
    {
        return RcSpeedGear::FAST;
    }
    return fallback;
}

RcOverrideChannels BuildRcOverrideChannels(
    const RcPwmCommand& command,
    std::uint16_t center_pwm)
{
    RcOverrideChannels channels;
    channels.fill(kRcNoChange);
    channels[0] = center_pwm;
    channels[1] = center_pwm;
    channels[2] = command.heave;
    channels[3] = command.yaw;
    channels[4] = command.surge;
    channels[5] = command.sway;
    return channels;
}

RcOverrideChannels BuildRcReleaseChannels()
{
    RcOverrideChannels channels;
    channels.fill(kRcRelease);
    return channels;
}
