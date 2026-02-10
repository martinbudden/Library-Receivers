#include "ReceiverVirtual.h"

#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
void test_receiver_switches()
{
    ReceiverVirtual receiver; // NOLINT(misc-const-correctness) false positive

    uint8_t switchIndex = 0;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_EQUAL(1, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 3);
    TEST_ASSERT_EQUAL(3, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));


    switchIndex = 1;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_EQUAL(1, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 3);
    TEST_ASSERT_EQUAL(3, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));

    switchIndex = 2;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_EQUAL(1, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 3);
    TEST_ASSERT_EQUAL(3, receiver.get_switch(switchIndex));
    receiver.set_switch(switchIndex, 0);
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
}

void test_receiver_controls()
{
    ReceiverVirtual receiver;

    ReceiverBase::controls_t controls = receiver.get_controls();
    TEST_ASSERT_EQUAL_FLOAT(0.0F, controls.throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, controls.roll);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, controls.pitch);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, controls.yaw);

    controls = { 2.0F, 3.0F, 4.0F, 5.0F };
    receiver.set_controls(controls);
    TEST_ASSERT_EQUAL_FLOAT(2.0F, controls.throttle);
    TEST_ASSERT_EQUAL_FLOAT(3.0F, controls.roll);
    TEST_ASSERT_EQUAL_FLOAT(4.0F, controls.pitch);
    TEST_ASSERT_EQUAL_FLOAT(5.0F, controls.yaw);
}

void test_receiver_auxiliary_channels()
{
    ReceiverVirtual receiver;

    uint8_t switchIndex = 0;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_LOW, receiver.get_auxiliary_channel(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_HIGH, receiver.get_auxiliary_channel(switchIndex));

    switchIndex = 1;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_LOW, receiver.get_auxiliary_channel(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_HIGH, receiver.get_auxiliary_channel(switchIndex));

    switchIndex = 2;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_LOW, receiver.get_auxiliary_channel(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_HIGH, receiver.get_auxiliary_channel(switchIndex));

    switchIndex = 3;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_LOW, receiver.get_auxiliary_channel(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_HIGH, receiver.get_auxiliary_channel(switchIndex));

    receiver.set_auxiliary_channel_pwm(2, 1200);
    TEST_ASSERT_EQUAL(1200, receiver.get_auxiliary_channel(2));
    TEST_ASSERT_EQUAL(1200, receiver.get_channel_pwm(2 + ReceiverBase::STICK_COUNT));
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_receiver_switches);
    RUN_TEST(test_receiver_controls);
    RUN_TEST(test_receiver_auxiliary_channels);

    UNITY_END();
}
