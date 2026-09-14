#ifndef _NODEHEXA_CONTROLLER_H_
#define _NODEHEXA_CONTROLLER_H_

#include <driver/uart.h>
#include <esp_log.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>
#include <functional>
#include <string>

#include "nodehexa_uart_protocol.h"

#define TAG "NodeHexaController"

class NodeHexaController {
public:
    using LowBatteryCallback = std::function<void(const std::string&)>;

    NodeHexaController();
    ~NodeHexaController();

    void Initialize();
    void SetLowBatteryCallback(LowBatteryCallback callback);
    cJSON* SendCommand(const std::string& command, int steps = 1);
    cJSON* SendPerformanceCommand(const std::string& performance);
    cJSON* GetStatus();
    cJSON* SendSpeedLevelCommand(int speed_level);

private:
    enum class ProtocolMode : uint8_t { Unknown, Legacy, V2 };

    void UartRxLoop();
    static void UartRxTask(void* arg);
    void HandleIncomingFrame(const nodehexa_uart::Frame& frame);
    void HandleIncomingEvent(cJSON* root);
    void HandleIncomingResponse(cJSON* root, const std::string& payload, uint16_t sequence,
                                bool is_v2);
    void NotifyLowBattery(const std::string& message);
    void ClearLowBatteryStateIfExplicitHealthy(const cJSON* root);
    void NegotiateProtocol();
    cJSON* QueueCommand(cJSON* command, bool stop = false);
    static void CommandTask(void* arg);
    void CommandLoop();
    struct CommandJob {
        char payload[nodehexa_uart::kMaxPayloadLength + 1];
        uint32_t id;
        uint32_t generation;
    };
    QueueHandle_t command_queue_ = nullptr;
    TaskHandle_t command_task_ = nullptr;
    std::atomic<uint32_t> job_id_{0};
    std::atomic<uint32_t> generation_{0};
    std::string last_result_;
    std::string peer_info_;
    cJSON* SendJsonCommandAndWait(cJSON* json_cmd, uint32_t generation);
    bool SendLegacyCommand(const std::string& payload);
    bool SendV2Frame(nodehexa_uart::MessageType type, uint8_t flags, uint16_t sequence,
                     const std::string& payload);
    uint16_t NextSequence();
    bool IsLowBatteryPayload(const cJSON* root) const;

    static constexpr int UART_TIMEOUT_MS = 1000;
    static constexpr int HELLO_TIMEOUT_MS = 500;

    TaskHandle_t uart_rx_task_handle_ = nullptr;
    EventGroupHandle_t response_event_group_ = nullptr;
    SemaphoreHandle_t command_mutex_ = nullptr;
    SemaphoreHandle_t state_mutex_ = nullptr;
    SemaphoreHandle_t tx_mutex_ = nullptr;
    nodehexa_uart::Parser parser_;
    std::string pending_response_;
    LowBatteryCallback low_battery_callback_;
    bool awaiting_response_ = false;
    uint16_t pending_sequence_ = 0;
    bool pending_v2_ = false;
    std::atomic<ProtocolMode> protocol_mode_{ProtocolMode::Unknown};
    std::atomic<uint16_t> next_sequence_{1};
    std::atomic<uint32_t> last_response_ms_{0};
    uint32_t last_heartbeat_ms_ = 0;
    bool low_battery_active_ = false;
    bool low_battery_notified_ = false;
};

#endif  // _NODEHEXA_CONTROLLER_H_
