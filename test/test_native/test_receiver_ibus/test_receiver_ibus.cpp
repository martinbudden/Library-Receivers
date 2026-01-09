#include "ReceiverIBUS.h"

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
    static SerialPort serialPort(SerialPort::uart_pins_t{}, 0, 0, ReceiverIBUS::DATA_BITS, ReceiverIBUS::STOP_BITS, ReceiverIBUS::PARITY);
    static ReceiverIBUS receiver(serialPort);

    receiver.setPacketEmpty();
    TEST_ASSERT_TRUE(receiver.isPacketEmpty());
    TEST_ASSERT_EQUAL(0, receiver.getPacketIndex());

    std::array<uint8_t, 32> data = {
        0x20, 0x40, 0xDB, 0x05, 0xDC, 0x05, 0x54, 0x05,
        0xDC, 0x05, 0xE8, 0x03, 0xD0, 0x07, 0xD2, 0x05,
        0xE8, 0x03, 0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05,
        0xDC, 0x05, 0xDC, 0x05, 0xDC, 0x05, 0x80, 0x4F
    };

    TEST_ASSERT_FALSE(receiver.onDataReceivedFromISR(data[0]));
    TEST_ASSERT_EQUAL(ReceiverIBUS::MODEL_IA6B, receiver.getModel());
    TEST_ASSERT_EQUAL(32, receiver.getSyncByte());
    TEST_ASSERT_EQUAL(32, receiver.getFrameSize());
    TEST_ASSERT_EQUAL(2, receiver.getChannelOffset());
    TEST_ASSERT_EQUAL(1, receiver.getPacketIndex());

    for (size_t ii = 1; ii < 30; ++ii) {
        const bool ret = receiver.onDataReceivedFromISR(data[ii]);
        TEST_ASSERT_FALSE(ret);
        TEST_ASSERT_EQUAL(ii + 1, receiver.getPacketIndex());
    }
    TEST_ASSERT_FALSE(receiver.onDataReceivedFromISR(data[30]));
    TEST_ASSERT_EQUAL(31, receiver.getPacketIndex());
    TEST_ASSERT_TRUE(receiver.onDataReceivedFromISR(data[31]));
    TEST_ASSERT_EQUAL(0, receiver.getPacketIndex());

    TEST_ASSERT_EQUAL(0x80, receiver.getPacket(30));
    TEST_ASSERT_EQUAL(0x4F, receiver.getPacket(31));

    TEST_ASSERT_TRUE(receiver.unpackPacket());
    TEST_ASSERT_EQUAL(0x4F80, receiver.getReceivedChecksum());
    TEST_ASSERT_EQUAL(0x4F80, receiver.calculateChecksum());

    TEST_ASSERT_EQUAL(0x05DB, receiver.getChannelPWM(0));
    TEST_ASSERT_EQUAL(0x05DC, receiver.getChannelPWM(1));
    TEST_ASSERT_EQUAL(0x0554, receiver.getChannelPWM(2));
    TEST_ASSERT_EQUAL(0x05DC, receiver.getChannelPWM(3));
    TEST_ASSERT_EQUAL(0x03E8, receiver.getChannelPWM(4));
    TEST_ASSERT_EQUAL(0x07D0, receiver.getChannelPWM(5));
    TEST_ASSERT_EQUAL(0x05D2, receiver.getChannelPWM(6));
    TEST_ASSERT_EQUAL(0x03E8, receiver.getChannelPWM(7));
    TEST_ASSERT_EQUAL(0x05DC, receiver.getChannelPWM(8));
    TEST_ASSERT_EQUAL(0x05DC, receiver.getChannelPWM(9));
    TEST_ASSERT_EQUAL(0x05DC, receiver.getChannelPWM(10));
    TEST_ASSERT_EQUAL(0x05DC, receiver.getChannelPWM(11));
    TEST_ASSERT_EQUAL(0x05DC, receiver.getChannelPWM(12));
    TEST_ASSERT_EQUAL(0x05DC, receiver.getChannelPWM(13));
 }

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-convert-member-functions-to-static,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_receiver_ibus);

    UNITY_END();
}
