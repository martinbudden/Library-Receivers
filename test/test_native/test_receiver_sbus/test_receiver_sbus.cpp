#include "ReceiverSBUS.h"

#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-convert-member-functions-to-static,readability-magic-numbers)
void test_receiver_sbus()
{
    static SerialPort serialPort(SerialPort::uart_pins_t{}, 0, 0, ReceiverSBUS::DATA_BITS, ReceiverSBUS::STOP_BITS, ReceiverSBUS::PARITY);
    static ReceiverSBUS receiver(serialPort);

    receiver.setPacketEmpty();
    TEST_ASSERT_TRUE(receiver.isPacketEmpty());
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-convert-member-functions-to-static,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_receiver_sbus);

    UNITY_END();
}
