#include "receiver_crsf.h"

#include <unity.h>

void setUp()
{
}

void tearDown()
{
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-convert-member-functions-to-static,readability-magic-numbers)
void test_receiver_crsf()
{
    static SerialPort serialPort(SerialPort::uart_pins_t{}, 0, 0, ReceiverCrsf::DATA_BITS, ReceiverCrsf::STOP_BITS, ReceiverCrsf::PARITY);
    static ReceiverCrsf receiver(serialPort);
    enum { CRC = 205 };
    ReceiverCrsf::packet_u packet = {
        .value = {
            .sync = ReceiverCrsf::CRSF_SYNC_BYTE,
            .length = 4, // length is length of type, payload, and CRC
            .type = ReceiverCrsf::FRAMETYPE_RC_CHANNELS_PACKED,
            .payload = { 01, 02, CRC }
        }
    };
    TEST_ASSERT_EQUAL(ReceiverCrsf::CRSF_SYNC_BYTE, packet.value.sync);
    TEST_ASSERT_EQUAL(4, packet.value.length);
    TEST_ASSERT_EQUAL(ReceiverCrsf::FRAMETYPE_RC_CHANNELS_PACKED, packet.value.type);
    TEST_ASSERT_EQUAL(CRC, packet.value.payload[2]);
    TEST_ASSERT_EQUAL(CRC, packet.value.payload[packet.value.length-2]);
    uint8_t crc = ReceiverCrsf::calculate_crc(0, packet.value.type);
    crc = ReceiverCrsf::calculate_crc(crc, packet.value.payload[0]);
    crc = ReceiverCrsf::calculate_crc(crc, packet.value.payload[1]);
    TEST_ASSERT_EQUAL(CRC, crc);



    for (uint8_t data : packet.data) {
        receiver.on_data_received_from_isr(data);
    }
    TEST_ASSERT_EQUAL(ReceiverCrsf::CRSF_SYNC_BYTE, receiver.get_packet_sync());
    TEST_ASSERT_EQUAL(4, receiver.get_packet_length());
    TEST_ASSERT_EQUAL(ReceiverCrsf::FRAMETYPE_RC_CHANNELS_PACKED, receiver.get_packet_type());
    TEST_ASSERT_EQUAL(CRC, receiver.get_received_crc());
    TEST_ASSERT_EQUAL(CRC, receiver.calculate_crc());
}

void test_receiver_bind_packet()
{
    static SerialPort serialPort(SerialPort::uart_pins_t{}, 0, 0, 0, 0, 0);
    static ReceiverCrsf receiver(serialPort);

    enum { COMMAND_CRC = 0x9E };
    enum { PACKET_CRC = 0xE8 };
    ReceiverCrsf::packet_u packet = {
        .value = {
            .sync = ReceiverCrsf::CRSF_SYNC_BYTE,
            .length = 7,
            .type = ReceiverCrsf::FRAMETYPE_COMMAND,
            .payload = {
                ReceiverCrsf::ADDRESS_CRSF_RECEIVER, 
                ReceiverCrsf::ADDRESS_FLIGHT_CONTROLLER, 
                ReceiverCrsf::COMMAND_SUBCMD_RX, 
                ReceiverCrsf::COMMAND_SUBCMD_RX_BIND,
                COMMAND_CRC,
                PACKET_CRC
            }
        }
    };
    for (uint8_t data : packet.data) {
        receiver.on_data_received_from_isr(data);
    }
    TEST_ASSERT_EQUAL(ReceiverCrsf::CRSF_SYNC_BYTE, receiver.get_packet_sync());
    TEST_ASSERT_EQUAL(7, receiver.get_packet_length());
    TEST_ASSERT_EQUAL(ReceiverCrsf::FRAMETYPE_COMMAND, receiver.get_packet_type());
    TEST_ASSERT_EQUAL(PACKET_CRC, receiver.get_received_crc());
    TEST_ASSERT_EQUAL(PACKET_CRC, receiver.calculate_crc());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-convert-member-functions-to-static,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_receiver_crsf);

    UNITY_END();
}
