#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "system_response.include/system_response.h"

extern "C" {
#include "flight/setpoint.h"
#include "pg/rates.h"
#include "pg/rx.h"
}

// Dummy functions (no intent to mock) to bypass linking errors.
extern "C" {
float getCosTiltAngle(void) { return 0; }
bool isSpooledUp(void) { return true; }
float pidGetPidFrequency() { return 1000; }
float pidGetDT() { return 1 / pidGetPidFrequency(); }
PG_REGISTER(rcControlsConfig_t, rcControlsConfig, PG_RC_CONTROLS_CONFIG, 0);

// We don't convert to any rates. The test focuses on normalized value (-1, 1).
float applyRatesCurve(const int, float rcCommandf) { return rcCommandf; }
}
// Dummy variables.
uint8_t armingFlags = 0;
uint16_t flightModeFlags = 0;

// Mock interface and their C function stubs
class MockInterface {
  public:
    MOCK_METHOD(float, getRcDeflection, (int axis), ());
} *g_mock = nullptr;

extern "C" {
float getRcDeflection(int axis) { return g_mock->getRcDeflection(axis); }
}

// Mocked variables
static controlRateConfig_t controlRateProfile;
controlRateConfig_t *currentControlRateProfile = &controlRateProfile;

using testing::StrictMock;

class SetpointTestBase : public ::testing::Test {
  public:
    void SetUp() override
    {
        g_mock = &mock;
        setpointInit();
    }
    void TearDown() override { g_mock = nullptr; }
    StrictMock<MockInterface> mock;
};

class SetpointBoostTest : public SetpointTestBase {
  public:
    void SetBoostParam(uint8_t boost, uint8_t cutoff)
    {
        controlRateProfile.setpoint_boost_gain[0] = boost;
        controlRateProfile.setpoint_boost_cutoff[0] = cutoff;
        setpointInitProfile();
    }
    std::tuple<int, int, float> CalcStepResponse()
    {
        // Output 0 for 1000 iterations
        EXPECT_CALL(mock, getRcDeflection(testing::_))
            .WillRepeatedly(testing::Return(0));
        for (int i = 0; i < 1000; ++i) {
            setpointUpdate();
        }

        // Output 1 for 1000 iterations and analyze the step response
        EXPECT_CALL(mock, getRcDeflection(testing::_))
            .WillRepeatedly(testing::Return(1));
        system_response::SystemResponse sr(
            [](void *) -> float {
                setpointUpdate();
                return getSetpoint(0);
            },
            1000, nullptr, 0.01);
        return {sr.GetSettleTime(), sr.GetOvershootTime(), sr.GetOvershoot()};
    }
};

TEST_F(SetpointBoostTest, Boost0)
{
    SetBoostParam(0, 0);
    auto [settle_time, overshoot_time, overshoot] = CalcStepResponse();

    // 10ms settle time and no overshoot
    EXPECT_LT(settle_time, 10);
    EXPECT_LT(overshoot, 0.001);

    std::cout << "Boost 0: " << settle_time << ", " << overshoot_time << ", "
              << overshoot << std::endl;
}

TEST_F(SetpointBoostTest, Boost90)
{
    SetBoostParam(90, 35);
    auto [settle_time, overshoot_time, overshoot] = CalcStepResponse();

    // 50ms settle time and 5% overshoot (estimate)
    EXPECT_LT(settle_time, 50);
    EXPECT_GT(overshoot, 0.05);

    std::cout << "Boost 90: " << settle_time << ", " << overshoot_time << ", "
              << overshoot << std::endl;
}

extern "C" float getDynamicDeadband(void);
class DynamicDeadbandTest : public SetpointTestBase {
  public:
    void SetDynamicDeadbandParam(uint8_t gain, uint8_t cutoff)
    {
        controlRateProfile.yaw_dynamic_deadband_gain = gain;
        controlRateProfile.yaw_dynamic_deadband_cutoff = cutoff;
        setpointInitProfile();
    }

    // ms = yaw stick centering speed (1.0 -> 0 in ms).
    std::tuple<int, int, float> CalcResponse(int ms)
    {
        // Output 1 for 1000 iterations
        EXPECT_CALL(mock, getRcDeflection(testing::_))
            .WillRepeatedly(testing::Return(1));
        for (int i = 0; i < 1000; ++i) {
            setpointUpdate();
        }

        EXPECT_CALL(mock, getRcDeflection(0))
            .WillRepeatedly(testing::Return(1));
        EXPECT_CALL(mock, getRcDeflection(1))
            .WillRepeatedly(testing::Return(1));
        EXPECT_CALL(mock, getRcDeflection(3))
            .WillRepeatedly(testing::Return(1));

        // Reduce to 0 in ms iterations.
        EXPECT_CALL(mock, getRcDeflection(2))
            .WillRepeatedly([&]()->float {
                static int count = 0;
                return count++ < ms ? 1 - (float)count/ms : 0;
            });
        step_response::StepResponse sr(
            [](void *) -> float {
                setpointUpdate();
                return getDynamicDeadband();
            },
            250, nullptr, 0.01);
        return {sr.GetSettleTime(), sr.GetOvershootTime(), sr.GetOvershoot()};
    }
};

TEST_F(DynamicDeadbandTest, Gain100)
{
    SetDynamicDeadbandParam(100, 30);
    auto [settle_time, overshoot_time, overshoot] = CalcResponse(75);

    // 20ms settle time and >10% max deadband (estimate)
    EXPECT_LT(settle_time, 75+20);
    EXPECT_GT(overshoot, 0.10);

    std::cout << "Gain 90: " << settle_time << ", " << overshoot_time << ", "
              << overshoot << std::endl;
}

TEST_F(DynamicDeadbandTest, Limit)
{
    SetDynamicDeadbandParam(200, 30);
    auto [settle_time, overshoot_time, overshoot] = CalcResponse(25);

    // 20ms settle time and max deadband capped by 20% limit
    EXPECT_LT(settle_time, 25+20);
    EXPECT_LT(overshoot, 0.25);

    std::cout << "Gain 90: " << settle_time << ", " << overshoot_time << ", "
              << overshoot << std::endl;
}
