#ifndef _NODEHEXA_CONTROLLER_H_
#define _NODEHEXA_CONTROLLER_H_

#include <cJSON.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <functional>
#include <string>

#define TAG "NodeHexaController"

class NodeHexaController {
public:
    using LowBatteryCallback = std::function<void(const std::string&)>;

    NodeHexaController();
    ~NodeHexaController();

    void Initialize();
    void SetLowBatteryCallback(LowBatteryCallback callback);
    cJSON* SendCommand(const std::string& command);
    cJSON* SendSpeedLevelCommand(int speed_level);

private:
    void UartRxLoop();
    static void UartRxTask(void* arg);
    void HandleIncomingFrame(const std::string& frame);
    void HandleIncomingEvent(cJSON* root);
    void HandleIncomingResponse(cJSON* root, const std::string& frame);
    void NotifyLowBattery(const std::string& message);
    void ClearLowBatteryState();
    cJSON* SendJsonCommandAndWait(cJSON* json_cmd);
    bool SendUartCommand(const std::string& command);
    int16_t CommandToMovementMode(const std::string& command);
    bool IsLowBatteryPayload(const cJSON* root) const;

    static constexpr int UART_TIMEOUT_MS = 1000;
    static constexpr int UART_BUFFER_SIZE = 256;

    TaskHandle_t uart_rx_task_handle_ = nullptr;
    EventGroupHandle_t response_event_group_ = nullptr;
    SemaphoreHandle_t command_mutex_ = nullptr;
    SemaphoreHandle_t state_mutex_ = nullptr;
    std::string pending_response_;
    LowBatteryCallback low_battery_callback_;
    bool awaiting_response_ = false;
    bool low_battery_active_ = false;
    bool low_battery_notified_ = false;
};

#endif // _NODEHEXA_CONTROLLER_H_ 