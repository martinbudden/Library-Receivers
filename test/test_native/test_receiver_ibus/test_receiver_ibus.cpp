#include "ReceiverIbus.h"

#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-convert-member-functions-to-static,readability-magic-numbers)
void test_receiver_ibus()
{
    static SerialPort serialPort(SerialPort::uart_pins_t{}, 0, 0, ReceiverIbus::DATA_BITS, ReceiverIbus::STOP_BITS, ReceiverIbus::PARITY);
    static ReceiverIbus receiver(serialPort);

    receiver.set_packet_empty();
    TEST_ASSERT_TRUE(receiver.is_packet_empty());
    TEST_ASSERT_EQUAL(0, receiver.get_packet_index());

    std::array<uint8_t, 32> data = {
        0x20, 0x40, 0xDB, 0x05, 0xDC, 0x05, 0x54, 0x05,
        0xDC, 0x05, 0xE8, 0x03, 0xD0, 0x07, 0xD2, 0x05,
        0xE8, 0x03, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05,
        0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0x80, 0x4F
    };

    TEST_ASSERT_FALSE(receiver.on_data_received_from_isr(data[0]));
    TEST_ASSERT_EQUAL(ReceiverIbus::MODEL_IA6B, receiver.getModel());
    TEST_ASSERT_EQUAL(32, receiver.get_sync_byte());
    TEST_ASSERT_EQUAL(32, receiver.get_frame_size());
    TEST_ASSERT_EQUAL(2, receiver.get_channel_offset());
    TEST_ASSERT_EQUAL(1, receiver.get_packet_index());

    for (size_t ii = 1; ii < 30; ++ii) {
        const bool ret = receiver.on_data_received_from_isr(data[ii]);
        TEST_ASSERT_FALSE(ret);
        TEST_ASSERT_EQUAL(ii + 1, receiver.get_packet_index());
    }
    TEST_ASSERT_FALSE(receiver.on_data_received_from_isr(data[30]));
    TEST_ASSERT_EQUAL(31, receiver.get_packet_index());
    TEST_ASSERT_TRUE(receiver.on_data_received_from_isr(data[31]));
    TEST_ASSERT_EQUAL(0, receiver.get_packet_index());

    TEST_ASSERT_EQUAL(0x80, receiver.get_packet(30));
    TEST_ASSERT_EQUAL(0x4F, receiver.get_packet(31));

    TEST_ASSERT_TRUE(receiver.unpack_packet());
    TEST_ASSERT_EQUAL(0x4F80, receiver.get_received_checksum());
    TEST_ASSERT_EQUAL(0x4F80, receiver.calculate_checksum());

    TEST_ASSERT_EQUAL(0x05DB, receiver.get_channel_pwm(0));
    TEST_ASSERT_EQUAL(0x05DC, receiver.get_channel_pwm(1));
    TEST_ASSERT_EQUAL(0x0554, receiver.get_channel_pwm(2));
    TEST_ASSERT_EQUAL(0x05DC, receiver.get_channel_pwm(3));
    TEST_ASSERT_EQUAL(0x03E8, receiver.get_channel_pwm(4));
    TEST_ASSERT_EQUAL(0x07D0, receiver.get_channel_pwm(5));
    TEST_ASSERT_EQUAL(0x05D2, receiver.get_channel_pwm(6));
    TEST_ASSERT_EQUAL(0x03E8, receiver.get_channel_pwm(7));
    TEST_ASSERT_EQUAL(0x05DC, receiver.get_channel_pwm(8));
    TEST_ASSERT_EQUAL(0x05DC, receiver.get_channel_pwm(9));
    TEST_ASSERT_EQUAL(0x05DC, receiver.get_channel_pwm(10));
    TEST_ASSERT_EQUAL(0x05DC, receiver.get_channel_pwm(11));
    TEST_ASSERT_EQUAL(0x05DC, receiver.get_channel_pwm(12));
    TEST_ASSERT_EQUAL(0x05DC, receiver.get_channel_pwm(13));
 }

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-convert-member-functions-to-static,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_receiver_ibus);

    UNITY_END();
}
