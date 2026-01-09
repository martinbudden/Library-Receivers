
#include "ESPNOW_Transceiver.h"
#include <cassert>
#include <cstring>

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
//#include <HardwareSerial.h>
#include <esp_log.h>
#include <esp_wifi.h>
#else
#define IRAM_ATTR
#endif

// see https://github.com/espressif/esp-idf/blob/v5.3.1/components/esp_wifi/include/esp_now.h

constexpr std::array<uint8_t, ESP_NOW_ETH_ALEN> ESPNOW_Transceiver::broadcastMacAddress {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/*!
Pointer to the transceiver used by the callback functions.
*/
ESPNOW_Transceiver* ESPNOW_Transceiver::transceiver;

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
/*!
Callback when data is sent.
*/
IRAM_ATTR void ESPNOW_Transceiver::onDataSent(const uint8_t* macAddress, esp_now_send_status_t status)
{
    (void)macAddress;
    // status can be ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL
    transceiver->_sendStatus = status;
}

/*!
Callback when data is received.

This called within an Interrupt Service Routine (ISR) in the high priority WiFi task, and so should not perform any lengthy operations.

Parameter `len` is `int` rather than `size_t` to match `esp_now_recv_cb_t` callback signature.
*/
#if defined(LIBRARY_RECEIVER_USE_ESPNOW_RECV_CB_MAC_ADDRESS) || !defined(LIBRARY_RECEIVER_USE_ESPNOW)
IRAM_ATTR void ESPNOW_Transceiver::onDataReceived(const uint8_t* macAddress, const uint8_t* data, int len)
{
#else
IRAM_ATTR void ESPNOW_Transceiver::onDataReceived(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    const uint8_t* macAddress = info->src_addr;
#endif
    if (!transceiver->isPrimaryPeerMacAddressSet()) {
        // If data is received when the primary peer MAC address is not yet set, it means we are in the binding process
        // So if check this data is not a broadcast packet and is not from the secondary peer
        if (!transceiver->macAddressIsBroadCastMacAddress(macAddress) && !transceiver->macAddressIsSecondaryPeerMacAddress(macAddress)) {
            transceiver->setPrimaryPeerMacAddress(macAddress);
        }
    }
    transceiver->copyReceivedDataToBuffer(macAddress, data, static_cast<size_t>(len));
}
#endif

ESPNOW_Transceiver::ESPNOW_Transceiver(const uint8_t* myMacAddress, uint8_t channel) :
    _channel(channel)
{
    transceiver = this;
    memcpy(&_myMacAddress[0], myMacAddress, ESP_NOW_ETH_ALEN);
#if defined(LIBRARY_RECEIVER_USE_ESPNOW) && defined(FRAMEWORK_USE_FREERTOS)
    _primaryDataReceivedQueue = xQueueCreateStatic(DATA_RECEIVED_QUEUE_LENGTH, sizeof(_primaryDataReceivedQueueItem), &_primaryDataReceivedQueueStorageArea[0], &_primaryDataReceivedQueueStatic);
    configASSERT(_primaryDataReceivedQueue);
    const UBaseType_t primaryMessageCount = uxQueueMessagesWaiting(_primaryDataReceivedQueue);
    assert(primaryMessageCount == 0);

    _secondaryDataReceivedQueue = xQueueCreateStatic(DATA_RECEIVED_QUEUE_LENGTH, sizeof(_secondaryDataReceivedQueueItem), &_secondaryDataReceivedQueueStorageArea[0], &_secondaryDataReceivedQueueStatic);
    configASSERT(_secondaryDataReceivedQueue);
    const UBaseType_t secondaryMessageCount = uxQueueMessagesWaiting(_secondaryDataReceivedQueue);
    assert(secondaryMessageCount == 0);
#endif
}

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
esp_err_t ESPNOW_Transceiver::init()
{
    static const char *TAG = "ESPNOW_Transceiver::init";

    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!!! esp_now_init failed: %s", esp_err_to_name(err));
        //Serial.printf("!!!! esp_now_init failed: 0x%X (0x%X)\r\n\r\n", err, err - ESP_ERR_ESPNOW_BASE);
        return err;
    }
    err = esp_wifi_set_channel(_channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!!! esp_wifi_set_channel failed: %s", esp_err_to_name(err));
        return err;
    }
    err = addBroadcastPeer(_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!!! addBroadcastPeer failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_now_register_recv_cb(ESPNOW_Transceiver::onDataReceived);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!!! esp_now_register_recv_cb failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_now_register_send_cb(ESPNOW_Transceiver::onDataSent);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!!! esp_now_register_send_cb failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t ESPNOW_Transceiver::init(received_data_t& received_data, const uint8_t* primaryMacAddress)
{
    //Serial.printf("ESPNOW_Transceiver::init received data: %x, %d\r\n", received_data.bufferPtr, received_data.bufferSize);
    esp_err_t err = init();
    if (err != ESP_OK) {
        return err;
    }

    received_data.len = 0;
    _peerData[PRIMARY_PEER].receivedDataPtr = &received_data;
    _peerData[PRIMARY_PEER].peer_info.channel = _channel;
    _peerData[PRIMARY_PEER].peer_info.encrypt = false;
    _peerCount = 2; // since we have already added the broadcast peer

    // Set the primary MAC address now, if it is provided
    // Otherwise it will be set from `onDataReceived` as part of the binding process
    if (primaryMacAddress != nullptr) {
        err = setPrimaryPeerMacAddress(primaryMacAddress);
        if (err != ESP_OK) {
            static const char *TAG = "ESPNOW_Transceiver::init";
            ESP_LOGE(TAG, "!!!! setPrimaryPeerMacAddress failed: %s", esp_err_to_name(err));
            //Serial.printf("!!!! ESPNOW_Transceiver::init setPrimaryPeerMacAddress failed: 0x%X (0x%X)\r\n\r\n", err, err - ESP_ERR_ESPNOW_BASE);
        }
    }
    return ESP_OK;
}

esp_err_t ESPNOW_Transceiver::addBroadcastPeer(uint8_t channel)
{
    // set receivedDataPtr to nullptr so no broadcast data is copied
    _peerData[BROADCAST_PEER].receivedDataPtr = nullptr;
    memcpy(_peerData[BROADCAST_PEER].peer_info.peer_addr, &broadcastMacAddress[0], ESP_NOW_ETH_ALEN);
    _peerData[BROADCAST_PEER].peer_info.channel = channel;
    _peerData[BROADCAST_PEER].peer_info.encrypt = false;

    const esp_err_t err = esp_now_add_peer(&_peerData[BROADCAST_PEER].peer_info);
    if (err != ESP_OK) {
        return err;
    }

    // don't inadvertently erase an already set primary or secondary peer
    if (_peerCount < 1) {
        _peerCount = 1;
    }
    return err;
}

esp_err_t ESPNOW_Transceiver::addSecondaryPeer(received_data_t& received_data, const uint8_t* macAddress)
{
    received_data.len = 0;
    _peerData[PEER_2].receivedDataPtr = &received_data;
    _peerData[PEER_2].peer_info.channel = _peerData[PRIMARY_PEER].peer_info.channel;
    _peerData[PEER_2].peer_info.encrypt = false;
    if (macAddress != nullptr) {
        memcpy(_peerData[PEER_2].peer_info.peer_addr, macAddress, ESP_NOW_ETH_ALEN);
    }

    const esp_err_t err = esp_now_add_peer(&_peerData[PEER_2].peer_info);
    if (err != ESP_OK) {
        static const char *TAG = "ESPNOW_Transceiver::addSecondayrPeer";
        ESP_LOGE(TAG, "!!!! esp_now_add_peer failed: %s", esp_err_to_name(err));
        return err;
    }

    _peerCount = 3; // includes the primary peer and the broadcast peer
    return ESP_OK;
}

IRAM_ATTR bool ESPNOW_Transceiver::isPrimaryPeerMacAddressSet() const
{
    return static_cast<bool>(_isPrimaryPeerMacAddressSet);
}

/*!
May be called within ISR.
*/
IRAM_ATTR bool ESPNOW_Transceiver::macAddressIsBroadCastMacAddress(const uint8_t* macAddress) const
{
    if (_peerCount > 0) {
        // check this is not the broadcast MAC address
        if (memcmp(macAddress, &broadcastMacAddress[0], ESP_NOW_ETH_ALEN) == 0) {
            return true;
        }
    }
    return false;
}

/*!
May be called within ISR.
*/
IRAM_ATTR bool ESPNOW_Transceiver::macAddressIsSecondaryPeerMacAddress(const uint8_t* macAddress) const
{
    if (_peerCount > 2) {
        // the secondary peer has been set, so compare the MAC address with that of the secondary peer.
        if (memcmp(macAddress, _peerData[SECONDARY_PEER].peer_info.peer_addr, ESP_NOW_ETH_ALEN) == 0) {
            return true;
        }
    }
    return false;
}

/*!
May be called within ISR.
*/
IRAM_ATTR esp_err_t ESPNOW_Transceiver::setPrimaryPeerMacAddress(const uint8_t* macAddress)
{
    memcpy(_peerData[PRIMARY_PEER].peer_info.peer_addr, macAddress, ESP_NOW_ETH_ALEN);

    if (_isPrimaryPeerMacAddressSet != 0) {
        return ESP_OK;
    }

    _isPrimaryPeerMacAddressSet = static_cast<int>(true);

    const esp_err_t err = esp_now_add_peer(&_peerData[PRIMARY_PEER].peer_info);
    return err;
}

/*!
This function is called from within onDataReceived, which is called within an ISR, so no printf error reporting
*/
IRAM_ATTR bool ESPNOW_Transceiver::copyReceivedDataToBuffer(const uint8_t* macAddress, const uint8_t* data, size_t len) // NOLINT(readability-make-member-function-const) false positive
{
    esp_now_peer_info_t peerInfo;
    const esp_err_t err = esp_now_get_peer(macAddress, &peerInfo);
    if (err == ESP_ERR_ESPNOW_NOT_FOUND) {
        // ignore any data sent from someone who is not a peer
        return false;
    }
    if (err != ESP_OK) {
        return false;
    }

    for (size_t ii = PRIMARY_PEER; ii < _peerCount; ++ii) {
        const auto& peerData = _peerData[ii];
        if (memcmp(macAddress, peerData.peer_info.peer_addr, ESP_NOW_ETH_ALEN) == 0) {
            // copy the received data into the _peerData buffer
            const size_t copyLength = std::min(len, peerData.receivedDataPtr->bufferSize); // so don't overwrite buffer
            memcpy(peerData.receivedDataPtr->bufferPtr, data, copyLength);
            // and set the _peerData length
            peerData.receivedDataPtr->len = copyLength;
            if (ii == PRIMARY_PEER) {
                ++_receivedPacketCount; // only count packets being sent to the primary peer
                const TickType_t tickCount = xTaskGetTickCount();
                _tickCountDelta = tickCount - _tickCountPrevious;
                _tickCountPrevious = tickCount;
                SIGNAL_PRIMARY_DATA_RECEIVED_FROM_ISR();
            } else if (ii == SECONDARY_PEER) {
                SIGNAL_SECONDARY_DATA_RECEIVED_FROM_ISR();
            }
            return true;
        }
    }
    return false;
}

esp_err_t ESPNOW_Transceiver::sendData(const uint8_t* data, size_t len) const
{
    //const uint8_t* ma = _transmitMacAddress;
    //Serial.printf("sendData MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", ma[0], ma[1], ma[2], ma[3], ma[4], ma[5]);
    //Serial.printf("sendData len:%d\r\n", len);
    assert(len < ESP_NOW_MAX_DATA_LEN); // 250
    if (!isPrimaryPeerMacAddressSet() || data == nullptr || len==0) {
        return ESP_FAIL;
    }
    const esp_err_t err = esp_now_send(_peerData[PRIMARY_PEER].peer_info.peer_addr, data, len);
    //if (err != ESP_OK) { Serial.printf("!!!! sendData err:0x%X (0x%X)\r\n\r\n", err, err - ESP_ERR_ESPNOW_BASE); }
    return err;
}

esp_err_t ESPNOW_Transceiver::sendDataSecondary(const uint8_t* data, size_t len) const
{
    //const uint8_t* ma = _peerData[SECONDARY_PEER].peer_info.peer_addr;
    //Serial.printf("sendDataSecondary MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", ma[0], ma[1], ma[2], ma[3], ma[4], ma[5]);
    //Serial.printf("sendDataSecondary len:%d\r\n", len);
    assert(len < ESP_NOW_MAX_DATA_LEN); // 250
    if (_peerCount < SECONDARY_PEER + 1 || data == nullptr || len==0) {
        return ESP_FAIL;
    }
    const esp_err_t err = esp_now_send(_peerData[SECONDARY_PEER].peer_info.peer_addr, data, len);
    //if (err != ESP_OK) { Serial.printf("!!!! sendDataSecondary err:0x%X (0x%x)\r\n\r\n", err, err - ESP_ERR_ESPNOW_BASE); }
    return err;
}

#endif
