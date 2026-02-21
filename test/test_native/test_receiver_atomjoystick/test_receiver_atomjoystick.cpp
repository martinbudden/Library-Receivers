#include "receiver_atom_joystick.h"

#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
void test_receiver_atom_joystick_auxiliary_channels()
{
    enum { AUXILIARY_CHANNEL_COUNT = 3};
    std::array<uint8_t, 6> mac_address;
    enum { WIFI_CHANNEL = 0 };

    static ReceiverAtomJoystick receiver(&mac_address[0], WIFI_CHANNEL);

    uint8_t switchIndex = 0;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_LOW, receiver.get_auxiliary_channel(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_GREATER_THAN(ReceiverBase::CHANNEL_MIDDLE, receiver.get_auxiliary_channel(switchIndex));

    switchIndex = 1;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_LOW, receiver.get_auxiliary_channel(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_GREATER_THAN(ReceiverBase::CHANNEL_MIDDLE, receiver.get_auxiliary_channel(switchIndex));

    switchIndex = 2;
    TEST_ASSERT_EQUAL(0, receiver.get_switch(switchIndex));
    TEST_ASSERT_EQUAL(ReceiverBase::CHANNEL_LOW, receiver.get_auxiliary_channel(switchIndex));
    receiver.set_switch(switchIndex, 1);
    TEST_ASSERT_GREATER_THAN(ReceiverBase::CHANNEL_MIDDLE, receiver.get_auxiliary_channel(switchIndex));
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_receiver_atom_joystick_auxiliary_channels);

    UNITY_END();
}
