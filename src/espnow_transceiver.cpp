
#include "espnow_transceiver.h"
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

constexpr std::array<uint8_t, ESP_NOW_ETH_ALEN> EspnowTransceiver::broadcastMac_address {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/*!
Pointer to the transceiver used by the callback functions.
*/
EspnowTransceiver* EspnowTransceiver::transceiver;

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
/*!
Callback when data is sent.
*/
IRAM_ATTR void EspnowTransceiver::on_data_sent(const uint8_t* mac_address, esp_now_send_status_t status)
{
    (void)mac_address;
    // status can be ESP_NOW_SEND_SUCCESS or ESP_NOW_SEND_FAIL
    transceiver->_send_status = status;
}

/*!
Callback when data is received.

This called within an Interrupt Service Routine (ISR) in the high priority WiFi task, and so should not perform any lengthy operations.

Parameter `len` is `int` rather than `size_t` to match `esp_now_recv_cb_t` callback signature.
*/
#if defined(LIBRARY_RECEIVER_USE_ESPNOW_RECV_CB_MAC_ADDRESS) || !defined(LIBRARY_RECEIVER_USE_ESPNOW)
IRAM_ATTR void EspnowTransceiver::on_data_received(const uint8_t* mac_address, const uint8_t* data, int len)
{
#else
IRAM_ATTR void EspnowTransceiver::on_data_received(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    const uint8_t* mac_address = info->src_addr;
#endif
    if (!transceiver->is_primary_peer_mac_addressSet()) {
        // If data is received when the primary peer MAC address is not yet set, it means we are in the binding process
        // So if check this data is not a broadcast packet and is not from the secondary peer
        if (!transceiver->mac_address_is_broadcast_mac_address(mac_address) && !transceiver->mac_address_is_secondary_peer_mac_address(mac_address)) {
            transceiver->set_primary_peer_mac_address(mac_address);
        }
    }
    transceiver->copy_received_data_to_buffer(mac_address, data, static_cast<size_t>(len));
}
#endif

EspnowTransceiver::EspnowTransceiver(const uint8_t* my_mac_address, uint8_t channel) :
    _channel(channel)
{
    transceiver = this;
    memcpy(&_my_mac_address[0], my_mac_address, ESP_NOW_ETH_ALEN);
#if defined(LIBRARY_RECEIVER_USE_ESPNOW) && defined(FRAMEWORK_USE_FREERTOS)
    _primary_data_received_queue = xQueueCreateStatic(DATA_RECEIVED_QUEUE_LENGTH, sizeof(_primary_data_received_queue_item), &_primary_data_received_queueStorageArea[0], &_primary_data_received_queue_static);
    configASSERT(_primary_data_received_queue);
    const UBaseType_t primaryMessageCount = uxQueueMessagesWaiting(_primary_data_received_queue);
    assert(primaryMessageCount == 0);

    _secondary_data_received_queue = xQueueCreateStatic(DATA_RECEIVED_QUEUE_LENGTH, sizeof(_secondary_data_received_queueItem), &_secondary_data_received_queueStorageArea[0], &_secondary_data_received_queue_static);
    configASSERT(_secondary_data_received_queue);
    const UBaseType_t secondaryMessageCount = uxQueueMessagesWaiting(_secondary_data_received_queue);
    assert(secondaryMessageCount == 0);
#endif
}

#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
esp_err_t EspnowTransceiver::init()
{
    static const char *TAG = "EspnowTransceiver::init";

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
    err = add_broadcast_peer(_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!!! add_broadcast_peer failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_now_register_recv_cb(EspnowTransceiver::on_data_received);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!!! esp_now_register_recv_cb failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_now_register_send_cb(EspnowTransceiver::on_data_sent);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "!!!! esp_now_register_send_cb failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t EspnowTransceiver::init(received_data_t& received_data, const uint8_t* primaryMac_address)
{
    //Serial.printf("EspnowTransceiver::init received data: %x, %d\r\n", received_data.bufferPtr, received_data.bufferSize);
    esp_err_t err = init();
    if (err != ESP_OK) {
        return err;
    }

    received_data.len = 0;
    _peer_data[PRIMARY_PEER].receivedDataPtr = &received_data;
    _peer_data[PRIMARY_PEER].peer_info.channel = _channel;
    _peer_data[PRIMARY_PEER].peer_info.encrypt = false;
    _peer_count = 2; // since we have already added the broadcast peer

    // Set the primary MAC address now, if it is provided
    // Otherwise it will be set from `on_data_received` as part of the binding process
    if (primaryMac_address != nullptr) {
        err = set_primary_peer_mac_address(primaryMac_address);
        if (err != ESP_OK) {
            static const char *TAG = "EspnowTransceiver::init";
            ESP_LOGE(TAG, "!!!! set_primary_peer_mac_address failed: %s", esp_err_to_name(err));
            //Serial.printf("!!!! EspnowTransceiver::init set_primary_peer_mac_address failed: 0x%X (0x%X)\r\n\r\n", err, err - ESP_ERR_ESPNOW_BASE);
        }
    }
    return ESP_OK;
}

esp_err_t EspnowTransceiver::add_broadcast_peer(uint8_t channel)
{
    // set receivedDataPtr to nullptr so no broadcast data is copied
    _peer_data[BROADCAST_PEER].receivedDataPtr = nullptr;
    memcpy(_peer_data[BROADCAST_PEER].peer_info.peer_addr, &broadcastMac_address[0], ESP_NOW_ETH_ALEN);
    _peer_data[BROADCAST_PEER].peer_info.channel = channel;
    _peer_data[BROADCAST_PEER].peer_info.encrypt = false;

    const esp_err_t err = esp_now_add_peer(&_peer_data[BROADCAST_PEER].peer_info);
    if (err != ESP_OK) {
        return err;
    }

    // don't inadvertently erase an already set primary or secondary peer
    if (_peer_count < 1) {
        _peer_count = 1;
    }
    return err;
}

esp_err_t EspnowTransceiver::add_secondary_peer(received_data_t& received_data, const uint8_t* mac_address)
{
    received_data.len = 0;
    _peer_data[PEER_2].receivedDataPtr = &received_data;
    _peer_data[PEER_2].peer_info.channel = _peer_data[PRIMARY_PEER].peer_info.channel;
    _peer_data[PEER_2].peer_info.encrypt = false;
    if (mac_address != nullptr) {
        memcpy(_peer_data[PEER_2].peer_info.peer_addr, mac_address, ESP_NOW_ETH_ALEN);
    }

    const esp_err_t err = esp_now_add_peer(&_peer_data[PEER_2].peer_info);
    if (err != ESP_OK) {
        static const char *TAG = "EspnowTransceiver::add_secondary_peer";
        ESP_LOGE(TAG, "!!!! esp_now_add_peer failed: %s", esp_err_to_name(err));
        return err;
    }

    _peer_count = 3; // includes the primary peer and the broadcast peer
    return ESP_OK;
}

IRAM_ATTR bool EspnowTransceiver::is_primary_peer_mac_addressSet() const
{
    return static_cast<bool>(_is_primary_peer_mac_address_set);
}

/*!
May be called within ISR.
*/
IRAM_ATTR bool EspnowTransceiver::mac_address_is_broadcast_mac_address(const uint8_t* mac_address) const
{
    if (_peer_count > 0) {
        // check this is not the broadcast MAC address
        if (memcmp(mac_address, &broadcastMac_address[0], ESP_NOW_ETH_ALEN) == 0) {
            return true;
        }
    }
    return false;
}

/*!
May be called within ISR.
*/
IRAM_ATTR bool EspnowTransceiver::mac_address_is_secondary_peer_mac_address(const uint8_t* mac_address) const
{
    if (_peer_count > 2) {
        // the secondary peer has been set, so compare the MAC address with that of the secondary peer.
        if (memcmp(mac_address, _peer_data[SECONDARY_PEER].peer_info.peer_addr, ESP_NOW_ETH_ALEN) == 0) {
            return true;
        }
    }
    return false;
}

/*!
May be called within ISR.
*/
IRAM_ATTR esp_err_t EspnowTransceiver::set_primary_peer_mac_address(const uint8_t* mac_address)
{
    memcpy(_peer_data[PRIMARY_PEER].peer_info.peer_addr, mac_address, ESP_NOW_ETH_ALEN);

    if (_is_primary_peer_mac_address_set != 0) {
        return ESP_OK;
    }

    _is_primary_peer_mac_address_set = static_cast<int>(true);

    const esp_err_t err = esp_now_add_peer(&_peer_data[PRIMARY_PEER].peer_info);
    return err;
}

/*!
This function is called from within on_data_received, which is called within an ISR, so no printf error reporting
*/
IRAM_ATTR bool EspnowTransceiver::copy_received_data_to_buffer(const uint8_t* mac_address, const uint8_t* data, size_t len) // NOLINT(readability-make-member-function-const) false positive
{
    esp_now_peer_info_t peerInfo;
    const esp_err_t err = esp_now_get_peer(mac_address, &peerInfo);
    if (err == ESP_ERR_ESPNOW_NOT_FOUND) {
        // ignore any data sent from someone who is not a peer
        return false;
    }
    if (err != ESP_OK) {
        return false;
    }

    for (size_t ii = PRIMARY_PEER; ii < _peer_count; ++ii) {
        const auto& peerData = _peer_data[ii];
        if (memcmp(mac_address, peerData.peer_info.peer_addr, ESP_NOW_ETH_ALEN) == 0) {
            // copy the received data into the _peer_data buffer
            const size_t copyLength = std::min(len, peerData.receivedDataPtr->bufferSize); // so don't overwrite buffer
            memcpy(peerData.receivedDataPtr->bufferPtr, data, copyLength);
            // and set the _peer_data length
            peerData.receivedDataPtr->len = copyLength;
            if (ii == PRIMARY_PEER) {
                ++_received_packet_count; // only count packets being sent to the primary peer
                const TickType_t tick_count = xTaskGetTickCount();
                _tick_count_delta = tick_count - _tick_count_previous;
                _tick_count_previous = tick_count;
                SIGNAL_PRIMARY_DATA_RECEIVED_FROM_ISR();
            } else if (ii == SECONDARY_PEER) {
                SIGNAL_SECONDARY_DATA_RECEIVED_FROM_ISR();
            }
            return true;
        }
    }
    return false;
}

esp_err_t EspnowTransceiver::send_data(const uint8_t* data, size_t len) const
{
    //const uint8_t* ma = _transmitMac_address;
    //Serial.printf("send_data MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", ma[0], ma[1], ma[2], ma[3], ma[4], ma[5]);
    //Serial.printf("send_data len:%d\r\n", len);
    assert(len < ESP_NOW_MAX_DATA_LEN); // 250
    if (!is_primary_peer_mac_addressSet() || data == nullptr || len==0) {
        return ESP_FAIL;
    }
    const esp_err_t err = esp_now_send(_peer_data[PRIMARY_PEER].peer_info.peer_addr, data, len);
    //if (err != ESP_OK) { Serial.printf("!!!! send_data err:0x%X (0x%X)\r\n\r\n", err, err - ESP_ERR_ESPNOW_BASE); }
    return err;
}

esp_err_t EspnowTransceiver::send_data_secondary(const uint8_t* data, size_t len) const
{
    //const uint8_t* ma = _peer_data[SECONDARY_PEER].peer_info.peer_addr;
    //Serial.printf("send_data_secondary MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", ma[0], ma[1], ma[2], ma[3], ma[4], ma[5]);
    //Serial.printf("send_data_secondary len:%d\r\n", len);
    assert(len < ESP_NOW_MAX_DATA_LEN); // 250
    if (_peer_count < SECONDARY_PEER + 1 || data == nullptr || len==0) {
        return ESP_FAIL;
    }
    const esp_err_t err = esp_now_send(_peer_data[SECONDARY_PEER].peer_info.peer_addr, data, len);
    //if (err != ESP_OK) { Serial.printf("!!!! send_data_secondary err:0x%X (0x%x)\r\n\r\n", err, err - ESP_ERR_ESPNOW_BASE); }
    return err;
}

#endif
