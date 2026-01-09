#include "ReceiverCRSF.h"

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
    static SerialPort serialPort(SerialPort::uart_pins_t{}, 0, 0, ReceiverCRSF::DATA_BITS, ReceiverCRSF::STOP_BITS, ReceiverCRSF::PARITY);
    static ReceiverCRSF receiver(serialPort);
    enum { CRC = 205 };
    ReceiverCRSF::packet_u packet = {
        .value = {
            .sync = ReceiverCRSF::CRSF_SYNC_BYTE,
            .length = 4, // length is length of type, payload, and CRC
            .type = ReceiverCRSF::FRAMETYPE_RC_CHANNELS_PACKED,
            .payload = { 01, 02, CRC }
        }
    };
    TEST_ASSERT_EQUAL(ReceiverCRSF::CRSF_SYNC_BYTE, packet.value.sync);
    TEST_ASSERT_EQUAL(4, packet.value.length);
    TEST_ASSERT_EQUAL(ReceiverCRSF::FRAMETYPE_RC_CHANNELS_PACKED, packet.value.type);
    TEST_ASSERT_EQUAL(CRC, packet.value.payload[2]);
    TEST_ASSERT_EQUAL(CRC, packet.value.payload[packet.value.length-2]);
    uint8_t crc = ReceiverCRSF::calculateCRC(0, packet.value.type);
    crc = ReceiverCRSF::calculateCRC(crc, packet.value.payload[0]);
    crc = ReceiverCRSF::calculateCRC(crc, packet.value.payload[1]);
    TEST_ASSERT_EQUAL(CRC, crc);



    for (uint8_t data : packet.data) {
        receiver.onDataReceivedFromISR(data);
    }
    TEST_ASSERT_EQUAL(ReceiverCRSF::CRSF_SYNC_BYTE, receiver.getPacketSync());
    TEST_ASSERT_EQUAL(4, receiver.getPacketLength());
    TEST_ASSERT_EQUAL(ReceiverCRSF::FRAMETYPE_RC_CHANNELS_PACKED, receiver.getPacketType());
    TEST_ASSERT_EQUAL(CRC, receiver.getReceivedCRC());
    TEST_ASSERT_EQUAL(CRC, receiver.calculateCRC());
}

void test_receiver_bind_packet()
{
    static SerialPort serialPort(SerialPort::uart_pins_t{}, 0, 0, 0, 0, 0);
    static ReceiverCRSF receiver(serialPort);

    enum { COMMAND_CRC = 0x9E };
    enum { PACKET_CRC = 0xE8 };
    ReceiverCRSF::packet_u packet = {
        .value = {
            .sync = ReceiverCRSF::CRSF_SYNC_BYTE,
            .length = 7,
            .type = ReceiverCRSF::FRAMETYPE_COMMAND,
            .payload = {
                ReceiverCRSF::ADDRESS_CRSF_RECEIVER, 
                ReceiverCRSF::ADDRESS_FLIGHT_CONTROLLER, 
                ReceiverCRSF::COMMAND_SUBCMD_RX, 
                ReceiverCRSF::COMMAND_SUBCMD_RX_BIND,
                COMMAND_CRC,
                PACKET_CRC
            }
        }
    };
    for (uint8_t data : packet.data) {
        receiver.onDataReceivedFromISR(data);
    }
    TEST_ASSERT_EQUAL(ReceiverCRSF::CRSF_SYNC_BYTE, receiver.getPacketSync());
    TEST_ASSERT_EQUAL(7, receiver.getPacketLength());
    TEST_ASSERT_EQUAL(ReceiverCRSF::FRAMETYPE_COMMAND, receiver.getPacketType());
    TEST_ASSERT_EQUAL(PACKET_CRC, receiver.getReceivedCRC());
    TEST_ASSERT_EQUAL(PACKET_CRC, receiver.calculateCRC());
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,misc-const-correctness,readability-convert-member-functions-to-static,readability-magic-numbers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_receiver_crsf);

    UNITY_END();
}
