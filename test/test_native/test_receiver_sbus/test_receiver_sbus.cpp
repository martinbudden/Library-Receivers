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
    static SerialPort serialPort(SerialPort::uart_pins_t{}, 0, 0, ReceiverSbus::DATA_BITS, ReceiverSbus::STOP_BITS, ReceiverSbus::PARITY);
    static ReceiverSbus receiver(serialPort);

    receiver.set_packet_empty();
    TEST_ASSERT_TRUE(receiver.is_packet_empty());
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-convert-member-functions-to-static,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_receiver_sbus);

    UNITY_END();
}
