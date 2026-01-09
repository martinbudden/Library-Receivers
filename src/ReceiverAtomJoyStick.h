#pragma once

#include "ESPNOW_Transceiver.h"
#include "ReceiverBase.h"


class ReceiverAtomJoyStick : public ReceiverBase {
public:
    ReceiverAtomJoyStick(const uint8_t* macAddress, uint8_t channel);
private:
    // ReceiverAtomJoyStick is not copyable or moveable
    ReceiverAtomJoyStick(const ReceiverAtomJoyStick&) = delete;
    ReceiverAtomJoyStick& operator=(const ReceiverAtomJoyStick&) = delete;
    ReceiverAtomJoyStick(ReceiverAtomJoyStick&&) = delete;
    ReceiverAtomJoyStick& operator=(ReceiverAtomJoyStick&&) = delete;
public:
    enum { MODE_SWITCH = 1, ALT_MODE_SWITCH = 2, SWITCH_COUNT = 3 }; // switch count includes MOTOR_ON_OFF_SWITCH
    enum { MODE_STABLE = 0, MODE_SPORT = 1 };
    enum { ALT_MODE_AUTO = 4, ALT_MODE_MANUAL = 5};
public:
    int32_t init();

    virtual int32_t WAIT_FOR_DATA_RECEIVED(uint32_t ticksToWait) override;
    virtual bool update(uint32_t tickCountDelta) override;
    virtual void getStickValues(float& throttle, float& roll, float& pitch, float& yaw) const override;
    virtual EUI_48_t getMyEUI() const override;
    virtual EUI_48_t getPrimaryPeerEUI() const override;
    virtual void broadcastMyEUI() const override;
    uint16_t getChannelPWM(size_t index) const override;

    ESPNOW_Transceiver& getESPNOW_Transceiver() { return _transceiver; }
    static int32_t ubyte4float_to_Q12dot4(const uint8_t f[4]);
private:
    // from AtomJoyStickReceiver
    inline bool isPacketEmpty() const { return _received_data.len == 0 ? true : false;  }
    inline void setPacketEmpty() { _received_data.len = 0; }
    enum { DEFAULT_BROADCAST_COUNT = 20, DEFAULT_BROADCAST_DELAY_MS = 50 };
    esp_err_t broadcastMyMacAddressForBinding(int broadcastCount, uint32_t broadcastDelayMs) const;
    esp_err_t broadcastMyMacAddressForBinding() const { return broadcastMyMacAddressForBinding(DEFAULT_BROADCAST_COUNT, DEFAULT_BROADCAST_DELAY_MS); }
    virtual bool unpackPacket() override;
    enum checkPacket_t { CHECK_PACKET, DONT_CHECK_PACKET };
    bool unpackPacket(checkPacket_t checkPacket);
    void resetSticks();
    void setDeadband(int32_t deadband);
    void setCurrentReadingsToBias();
    float normalizedStick(size_t stickIndex) const;
private:
    struct stick_t {
        int32_t rawQ12dot4 {0};
        int32_t biasQ12dot4 {0};
        int32_t deadbandQ12dot4 {16}; // last 4 bits of number
    };
private:
    ESPNOW_Transceiver _transceiver;
    ESPNOW_Transceiver::received_data_t _received_data;
    uint32_t _receivedPacketCount {0};
    std::array<stick_t, STICK_COUNT> _sticks {};
    enum { PACKET_SIZE = 25 };
    uint8_t _packet[PACKET_SIZE] {};
    uint8_t _biasIsSet {false}; //NOTE: if `bool` type is used here then `getMode()` sometimes returns incorrect value
    uint8_t _mode {0};
    uint8_t _altMode {0};
    uint8_t _armButton {0};
    uint8_t _flipButton {0};
    uint8_t _proactiveFlag {0};
};
