# pragma once

#include <array>
#include <cstddef>
#include <cstdint>


#if defined(LIBRARY_RECEIVER_USE_ESPNOW)

#include <esp_attr.h>
#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#else

typedef int esp_err_t;
static constexpr int8_t ESP_OK=0;
static constexpr int8_t ESP_FAIL=-1;
enum esp_now_send_status_t {ESP_NOW_SEND_SUCCESS = 0, ESP_NOW_SEND_FAIL};
static constexpr uint8_t WIFI_IF_STA = 1;
static constexpr uint8_t WIFI_IF_AP = 2;
static constexpr uint8_t ESP_NOW_ETH_ALEN = 6;
static constexpr uint8_t ESP_NOW_KEY_LEN = 16;

struct esp_now_peer_info_t {
    uint8_t peer_addr[ESP_NOW_ETH_ALEN];
    uint8_t lmk[ESP_NOW_KEY_LEN];
    uint8_t channel;
    uint8_t ifidx;
    bool encrypt;
    void *priv;
};

#endif // LIBRARY_RECEIVER_USE_ESPNOW

class EspnowTransceiver {
public:
    static constexpr uint8_t BROADCAST_PEER = 0;
    static constexpr uint8_t PRIMARY_PEER = 1;
    static constexpr uint8_t SECONDARY_PEER = 2;
    static constexpr uint8_t PEER_2 = 2;
    static constexpr uint8_t PEER_3 = 4;
    static constexpr uint8_t MAX_PEER_COUNT = 4;
    struct received_data_t {
        inline received_data_t(uint8_t* aBufferPtr, size_t aBufferSize) : bufferPtr(aBufferPtr), bufferSize(aBufferSize), len(0) {}
        uint8_t* bufferPtr;
        size_t bufferSize;
        size_t len;
    };
    struct peer_data_t {
        esp_now_peer_info_t peer_info { .peer_addr={0,0,0,0,0,0}, .lmk={0}, .channel=0, .ifidx=WIFI_IF_STA, .encrypt=false, .priv=nullptr };
        received_data_t* receivedDataPtr {nullptr};
    };
public:
    EspnowTransceiver(const uint8_t* my_mac_address, uint8_t channel);
    // !!NOTE: all references passed to init() and add_secondary_peer() must be static or allocated, ie they must not be local variables on the stack
    esp_err_t init(received_data_t& received_data, const uint8_t* transmitMac_address);
    esp_err_t add_secondary_peer(received_data_t& received_data, const uint8_t* mac_address);
    inline const uint8_t* my_mac_address() const { return &_my_mac_address[0]; }
    // sends data to the primary peer,
    esp_err_t send_data(const uint8_t* data, size_t len) const;
    // sends data to the secondary peer,
    esp_err_t send_data_secondary(const uint8_t* data, size_t len) const;
    bool is_primary_peer_mac_addressSet() const;
    const uint8_t* get_primary_peer_mac_address() const { return _peer_data[PRIMARY_PEER].peer_info.peer_addr; }
    inline uint8_t get_broadcast_channel() const { return _peer_data[BROADCAST_PEER].peer_info.channel; }
    esp_err_t broadcast_data(const uint8_t* data, size_t len) const {
#if defined(LIBRARY_RECEIVER_USE_ESPNOW)
         return esp_now_send(_peer_data[BROADCAST_PEER].peer_info.peer_addr, data, len);
#else
        (void)data; (void)len; return -1;
#endif
    }
    inline uint32_t get_received_packet_count() const { return _received_packet_count; }
    inline uint32_t get_tick_count_delta() const { return _tick_count_delta; }
    inline uint32_t get_tick_count_delta_and_reset() { const uint32_t tick_count_delta = _tick_count_delta; _tick_count_delta = 0; return tick_count_delta; }
private:
    esp_err_t init();
    esp_err_t add_broadcast_peer(uint8_t channel);
    // when data is received the copy function is called to copy the received data into the client's buffer
    bool copy_received_data_to_buffer(const uint8_t* mac_address, const uint8_t* data, size_t len);
    bool mac_address_is_broadcast_mac_address(const uint8_t* mac_address) const;
    bool mac_address_is_secondary_peer_mac_address(const uint8_t* mac_address) const;
    esp_err_t set_primary_peer_mac_address(const uint8_t* mac_address);
private:
    static void on_data_sent(const uint8_t* mac_address, esp_now_send_status_t status);
#if defined(LIBRARY_RECEIVER_USE_ESPNOW_RECV_CB_MAC_ADDRESS) || !defined(LIBRARY_RECEIVER_USE_ESPNOW)
    static void on_data_received(const uint8_t* mac_address, const uint8_t* data, int len); // len is int rather than size_t to match esp_now_recv_cb_t callback signature
#else
    static void on_data_received(const esp_now_recv_info_t* info, const uint8_t* data, int len); // len is int rather than size_t to match esp_now_recv_cb_t callback signature
#endif
private:
    static const std::array<uint8_t, ESP_NOW_ETH_ALEN> broadcastMac_address;
    static EspnowTransceiver* transceiver; // alias of `this` to be used in on_data_sent and on_data_received callback functions
    // tick counts are used for instrumentation
    uint32_t _tick_count_previous {0};
    uint32_t _tick_count_delta {0};
    uint32_t _received_packet_count {0}; //!< used to check for dropped packets
    // by default the transceiver has two peers, the broadcast peer and the primary peer
    uint32_t _is_primary_peer_mac_address_set {false}; // this is uint32_t rather than bool because using bool caused strange bugs
    uint32_t _peer_count {0};
    uint32_t _channel;
    std::array<peer_data_t, MAX_PEER_COUNT> _peer_data;
    esp_now_send_status_t _send_status {ESP_NOW_SEND_SUCCESS};
    std::array<uint8_t, ESP_NOW_ETH_ALEN + 2> _my_mac_address {0, 0, 0, 0, 0, 0, 0, 0};

#if defined(LIBRARY_RECEIVER_USE_ESPNOW) && defined(FRAMEWORK_USE_FREERTOS)
    enum { DATA_RECEIVED_QUEUE_LENGTH = 1}; // length MUST be 1 to be used by xQueueOverwriteFromISR

    uint32_t _primary_data_received_queue_item {}; // this is just a dummy item whose value is not used
    BaseType_t _primary_data_received_queue_higher_priority_task_woken = pdFALSE;
    std::array<uint8_t, DATA_RECEIVED_QUEUE_LENGTH * sizeof(_primary_data_received_queue_item)> _primary_data_received_queueStorageArea {};
    StaticQueue_t _primary_data_received_queue_static {};
    QueueHandle_t _primary_data_received_queue {};

    uint32_t _secondary_data_received_queueItem {}; // this is just a dummy item whose value is not used
    BaseType_t _secondary_data_received_queueHigherPriorityTaskWoken = pdFALSE;
    std::array<uint8_t, DATA_RECEIVED_QUEUE_LENGTH * sizeof(_secondary_data_received_queueItem)> _secondary_data_received_queueStorageArea {};
    StaticQueue_t _secondary_data_received_queue_static {};
    QueueHandle_t _secondary_data_received_queue {};
public:
    inline int32_t WAIT_FOR_PRIMARY_DATA_RECEIVED() { return xQueueReceive(_primary_data_received_queue, &_primary_data_received_queue_item, portMAX_DELAY); }
    inline int32_t WAIT_FOR_PRIMARY_DATA_RECEIVED(uint32_t ticksToWait) { return xQueueReceive(_primary_data_received_queue, &_primary_data_received_queue_item, ticksToWait); }
    //inline void SIGNAL_PRIMARY_DATA_RECEIVED_FROM_ISR() { xQueueSendFromISR(_primary_data_received_queue, &_primary_data_received_queue_item, nullptr); }
    inline void SIGNAL_PRIMARY_DATA_RECEIVED_FROM_ISR() {
        _primary_data_received_queue_higher_priority_task_woken = pdFALSE;
        xQueueOverwriteFromISR(_primary_data_received_queue, &_primary_data_received_queue_item, &_primary_data_received_queue_higher_priority_task_woken);
        portYIELD_FROM_ISR(_primary_data_received_queue_higher_priority_task_woken); // or portEND_SWITCHING_ISR() depending on the port.
    }
    inline int32_t WAIT_FOR_SECONDARY_DATA_RECEIVED() { return xQueueReceive(_secondary_data_received_queue, &_secondary_data_received_queueItem, portMAX_DELAY); }
    inline int32_t WAIT_FOR_SECONDARY_DATA_RECEIVED(uint32_t ticksToWait) { return xQueueReceive(_secondary_data_received_queue, &_secondary_data_received_queueItem, ticksToWait); }
    //inline void SIGNAL_SECONDARY_DATA_RECEIVED_FROM_ISR() { xQueueSendFromISR(_secondary_data_received_queue, &_secondary_data_received_queueItem, nullptr); }
    inline void SIGNAL_SECONDARY_DATA_RECEIVED_FROM_ISR() {
        _secondary_data_received_queueHigherPriorityTaskWoken = pdFALSE;
        xQueueOverwriteFromISR(_secondary_data_received_queue, &_secondary_data_received_queueItem, &_secondary_data_received_queueHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(_secondary_data_received_queueHigherPriorityTaskWoken); // or portEND_SWITCHING_ISR() depending on the port.
    }
#else
public:
    inline int32_t WAIT_FOR_PRIMARY_DATA_RECEIVED() { return 0; }
    inline int32_t WAIT_FOR_PRIMARY_DATA_RECEIVED(uint32_t ticksToWait) { (void)ticksToWait; return 0; }
    inline void SIGNAL_PRIMARY_DATA_RECEIVED_FROM_ISR() {}
    inline int32_t WAIT_FOR_SECONDARY_DATA_RECEIVED() { return 0; }
    inline int32_t WAIT_FOR_SECONDARY_DATA_RECEIVED(uint32_t ticksToWait) { (void)ticksToWait; return 0; }
    inline void SIGNAL_SECONDARY_DATA_RECEIVED_FROM_ISR() {}
#endif
};
