#include <gtest/gtest.h>

#include <scheduler/rc_command_mapper.h>

TEST(RcCommandMapperTest, MapsThreeGearsSymmetricallyAroundCenter)
{
    RcCommandMapper mapper;
    RcNormalizedCommand command;
    command.surge = 1.0;
    command.sway = -1.0;

    command.gear = RcSpeedGear::SLOW;
    RcPwmCommand pwm = mapper.map(command);
    EXPECT_EQ(1650, pwm.surge);
    EXPECT_EQ(1350, pwm.sway);

    command.gear = RcSpeedGear::MEDIUM;
    pwm = mapper.map(command);
    EXPECT_EQ(1800, pwm.surge);
    EXPECT_EQ(1200, pwm.sway);

    command.gear = RcSpeedGear::FAST;
    pwm = mapper.map(command);
    EXPECT_EQ(1900, pwm.surge);
    EXPECT_EQ(1100, pwm.sway);
}

TEST(RcCommandMapperTest, ClampsInputAndSupportsAxisDirection)
{
    RcCommandMapperConfig config;
    config.yaw_direction = -1;
    RcCommandMapper mapper(config);

    RcNormalizedCommand command;
    command.gear = RcSpeedGear::FAST;
    command.surge = 3.0;
    command.yaw = 0.5;

    const RcPwmCommand pwm = mapper.map(command);
    EXPECT_EQ(1900, pwm.surge);
    EXPECT_EQ(1300, pwm.yaw);
}

TEST(RcCommandMapperTest, AppliesInputDeadband)
{
    RcCommandMapper mapper;
    RcNormalizedCommand command;
    command.gear = RcSpeedGear::FAST;
    command.surge = 0.01;

    EXPECT_EQ(1500, mapper.map(command).surge);
}

TEST(RcCommandMapperTest, BuildsMavrosOverrideChannelOrder)
{
    RcPwmCommand command;
    command.heave = 1510;
    command.yaw = 1520;
    command.surge = 1600;
    command.sway = 1400;

    const RcOverrideChannels channels = BuildRcOverrideChannels(command, 1500);
    EXPECT_EQ(1500, channels[0]);
    EXPECT_EQ(1500, channels[1]);
    EXPECT_EQ(1510, channels[2]);
    EXPECT_EQ(1520, channels[3]);
    EXPECT_EQ(1600, channels[4]);
    EXPECT_EQ(1400, channels[5]);

    for (std::size_t index = 6; index < channels.size(); ++index)
    {
        EXPECT_EQ(65535, channels[index]);
    }
}

TEST(RcCommandMapperTest, ReleasesAllMavrosOverrideChannels)
{
    const RcOverrideChannels channels = BuildRcReleaseChannels();
    for (std::uint16_t channel : channels)
    {
        EXPECT_EQ(0, channel);
    }
}
