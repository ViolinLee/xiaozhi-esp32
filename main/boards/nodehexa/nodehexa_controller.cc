#include "nodehexa_controller.h"

#include <cstring>
#include <utility>

namespace {

constexpr EventBits_t kResponseReadyBit = BIT0;
constexpr const char* kLowBatteryProtectCode = "LOW_BATTERY_PROTECT";
constexpr const char* kLowBatteryMessage = "电量低，请关闭电源后进行充电！";
constexpr const char* kCommandBusyMessage = "串口忙，正在等待上一条命令响应";
constexpr const char* kTimeoutMessage = "等待六足主板响应超时";
constexpr uint32_t kUartTaskStackSize = 4096;
constexpr UBaseType_t kUartTaskPriority = 5;

bool JsonStringEquals(const cJSON* root, const char* key, const char* expected) {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(root, key);
    return cJSON_IsString(item) && item->valuestring != nullptr && strcmp(item->valuestring, expected) == 0;
}

std::string GetJsonString(const cJSON* root, const char* key, const char* fallback = "") {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (cJSON_IsString(item) && item->valuestring != nullptr) {
        return item->valuestring;
    }
    return fallback;
}

bool ContainsLowBatteryText(const std::string& text) {
    return text.find("电量低") != std::string::npos || text.find("low battery") != std::string::npos;
}

cJSON* CreateResultJson(const char* status, const char* message, const char* code = nullptr) {
    cJSON* result = cJSON_CreateObject();
    cJSON_AddStringToObject(result, "status", status);
    if (message != nullptr) {
        cJSON_AddStringToObject(result, "message", message);
    }
    if (code != nullptr) {
        cJSON_AddStringToObject(result, "code", code);
    }
    return result;
}

}  // namespace

NodeHexaController::NodeHexaController() {
    command_mutex_ = xSemaphoreCreateMutex();
    state_mutex_ = xSemaphoreCreateMutex();
    response_event_group_ = xEventGroupCreate();
    ESP_LOGI(TAG, "NodeHexaController 构造函数");
}

NodeHexaController::~NodeHexaController() {
    if (uart_rx_task_handle_ != nullptr) {
        vTaskDelete(uart_rx_task_handle_);
        uart_rx_task_handle_ = nullptr;
    }
    if (response_event_group_ != nullptr) {
        vEventGroupDelete(response_event_group_);
        response_event_group_ = nullptr;
    }
    if (command_mutex_ != nullptr) {
        vSemaphoreDelete(command_mutex_);
        command_mutex_ = nullptr;
    }
    if (state_mutex_ != nullptr) {
        vSemaphoreDelete(state_mutex_);
        state_mutex_ = nullptr;
    }
    ESP_LOGI(TAG, "NodeHexaController 析构函数");
}

void NodeHexaController::Initialize() {
    if (uart_rx_task_handle_ != nullptr) {
        return;
    }
    ESP_LOGI(TAG, "初始化 NodeHexaController");
    xTaskCreate(NodeHexaController::UartRxTask,
                "nodehexa_uart_rx",
                kUartTaskStackSize,
                this,
                kUartTaskPriority,
                &uart_rx_task_handle_);
}

void NodeHexaController::SetLowBatteryCallback(LowBatteryCallback callback) {
    if (state_mutex_ != nullptr && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        low_battery_callback_ = std::move(callback);
        xSemaphoreGive(state_mutex_);
    }
}

cJSON* NodeHexaController::SendCommand(const std::string& command) {
    ESP_LOGI(TAG, "发送命令: %s", command.c_str());

    const int16_t movement_mode = CommandToMovementMode(command);
    if (movement_mode < 0) {
        ESP_LOGE(TAG, "未知命令: %s", command.c_str());
        return CreateResultJson("error", "未知命令");
    }

    cJSON* json_cmd = cJSON_CreateObject();
    cJSON_AddNumberToObject(json_cmd, "movementMode", movement_mode);

    cJSON* result = SendJsonCommandAndWait(json_cmd);
    cJSON_AddStringToObject(result, "command", command.c_str());
    if (cJSON_GetObjectItemCaseSensitive(result, "movementMode") == nullptr) {
        cJSON_AddNumberToObject(result, "movementMode", movement_mode);
    }
    return result;
}

cJSON* NodeHexaController::SendSpeedLevelCommand(int speed_level) {
    ESP_LOGI(TAG, "发送速度等级命令: %d", speed_level);

    if (speed_level < 0 || speed_level > 3) {
        ESP_LOGE(TAG, "无效的速度等级: %d", speed_level);
        return CreateResultJson("error", "无效的速度等级，范围应为0-3");
    }

    cJSON* json_cmd = cJSON_CreateObject();
    cJSON_AddNumberToObject(json_cmd, "speedLevel", speed_level);

    cJSON* result = SendJsonCommandAndWait(json_cmd);
    cJSON_AddStringToObject(result, "command", "speedLevel");
    if (cJSON_GetObjectItemCaseSensitive(result, "speedLevel") == nullptr) {
        cJSON_AddNumberToObject(result, "speedLevel", speed_level);
    }
    return result;
}

void NodeHexaController::UartRxTask(void* arg) {
    auto* self = static_cast<NodeHexaController*>(arg);
    self->UartRxLoop();
}

void NodeHexaController::UartRxLoop() {
    std::string buffer;
    buffer.reserve(UART_BUFFER_SIZE);
    bool frame_started = false;

    while (true) {
        uint8_t byte = 0;
        const int read_len = uart_read_bytes(UART_NUM_1, &byte, 1, pdMS_TO_TICKS(20));
        if (read_len <= 0) {
            continue;
        }

        const char c = static_cast<char>(byte);
        if (!frame_started) {
            if (c == '$') {
                frame_started = true;
                buffer.clear();
            }
            continue;
        }

        if (c == '\n' || c == '\r') {
            if (!buffer.empty()) {
                HandleIncomingFrame(buffer);
            }
            buffer.clear();
            frame_started = false;
            continue;
        }

        if (buffer.size() >= UART_BUFFER_SIZE - 1) {
            ESP_LOGW(TAG, "UART帧过长，丢弃当前帧");
            buffer.clear();
            frame_started = false;
            continue;
        }

        buffer.push_back(c);
    }
}

void NodeHexaController::HandleIncomingFrame(const std::string& frame) {
    ESP_LOGD(TAG, "UART接收帧: %s", frame.c_str());

    cJSON* root = cJSON_Parse(frame.c_str());
    if (root == nullptr || !cJSON_IsObject(root)) {
        if (root != nullptr) {
            cJSON_Delete(root);
        }
        ESP_LOGW(TAG, "收到无法解析的UART数据: %s", frame.c_str());
        return;
    }

    if (cJSON_GetObjectItemCaseSensitive(root, "event") != nullptr) {
        HandleIncomingEvent(root);
    } else if (cJSON_GetObjectItemCaseSensitive(root, "status") != nullptr) {
        HandleIncomingResponse(root, frame);
    } else {
        ESP_LOGW(TAG, "收到未知UART消息: %s", frame.c_str());
    }

    cJSON_Delete(root);
}

void NodeHexaController::HandleIncomingEvent(cJSON* root) {
    if (IsLowBatteryPayload(root)) {
        const std::string message = GetJsonString(root, "message", kLowBatteryMessage);
        ESP_LOGW(TAG, "收到六足主板低电量事件: %s", message.c_str());
        NotifyLowBattery(message);
        return;
    }

    const std::string event_name = GetJsonString(root, "event", "unknown");
    ESP_LOGI(TAG, "收到异步事件: %s", event_name.c_str());
}

void NodeHexaController::HandleIncomingResponse(cJSON* root, const std::string& frame) {
    if (IsLowBatteryPayload(root)) {
        const std::string message = GetJsonString(root, "message", kLowBatteryMessage);
        ESP_LOGW(TAG, "收到六足主板低电量响应: %s", message.c_str());
        NotifyLowBattery(message);
    } else if (JsonStringEquals(root, "status", "success")) {
        ClearLowBatteryState();
    }

    bool delivered = false;
    if (state_mutex_ != nullptr && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (awaiting_response_) {
            pending_response_ = frame;
            delivered = true;
        }
        xSemaphoreGive(state_mutex_);
    }

    if (delivered) {
        xEventGroupSetBits(response_event_group_, kResponseReadyBit);
    } else {
        ESP_LOGW(TAG, "收到未匹配请求的响应: %s", frame.c_str());
    }
}

void NodeHexaController::NotifyLowBattery(const std::string& message) {
    LowBatteryCallback callback;
    bool should_notify = false;
    const std::string final_message = message.empty() ? kLowBatteryMessage : message;

    if (state_mutex_ != nullptr && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        low_battery_active_ = true;
        if (!low_battery_notified_) {
            low_battery_notified_ = true;
            should_notify = true;
            callback = low_battery_callback_;
        }
        xSemaphoreGive(state_mutex_);
    }

    if (should_notify && callback) {
        ESP_LOGW(TAG, "触发低电量主动提醒: %s", final_message.c_str());
        callback(final_message);
    }
}

void NodeHexaController::ClearLowBatteryState() {
    if (state_mutex_ != nullptr && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        low_battery_active_ = false;
        low_battery_notified_ = false;
        xSemaphoreGive(state_mutex_);
    }
}

cJSON* NodeHexaController::SendJsonCommandAndWait(cJSON* json_cmd) {
    if (command_mutex_ == nullptr || state_mutex_ == nullptr || response_event_group_ == nullptr) {
        cJSON_Delete(json_cmd);
        return CreateResultJson("error", "控制器尚未完成初始化");
    }

    char* json_str = cJSON_PrintUnformatted(json_cmd);
    const std::string uart_command = std::string("$") + (json_str != nullptr ? json_str : "{}") + "\n";
    if (json_str != nullptr) {
        cJSON_free(json_str);
    }
    cJSON_Delete(json_cmd);

    if (xSemaphoreTake(command_mutex_, pdMS_TO_TICKS(UART_TIMEOUT_MS)) != pdTRUE) {
        return CreateResultJson("error", kCommandBusyMessage);
    }

    xEventGroupClearBits(response_event_group_, kResponseReadyBit);
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        pending_response_.clear();
        awaiting_response_ = true;
        xSemaphoreGive(state_mutex_);
    }

    cJSON* result = nullptr;
    if (!SendUartCommand(uart_command)) {
        result = CreateResultJson("error", "UART发送失败");
        goto cleanup;
    }

    {
        const EventBits_t bits = xEventGroupWaitBits(response_event_group_,
                                                     kResponseReadyBit,
                                                     pdTRUE,
                                                     pdFALSE,
                                                     pdMS_TO_TICKS(UART_TIMEOUT_MS));
        if ((bits & kResponseReadyBit) == 0) {
            result = CreateResultJson("error", kTimeoutMessage);
            goto cleanup;
        }
    }

    {
        std::string response;
        if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
            response = pending_response_;
            pending_response_.clear();
            xSemaphoreGive(state_mutex_);
        }

        if (response.empty()) {
            result = CreateResultJson("error", kTimeoutMessage);
            goto cleanup;
        }

        result = cJSON_Parse(response.c_str());
        if (result == nullptr || !cJSON_IsObject(result)) {
            if (result != nullptr) {
                cJSON_Delete(result);
            }
            result = CreateResultJson("error", "收到无法解析的六足主板响应");
            cJSON_AddStringToObject(result, "rawResponse", response.c_str());
            goto cleanup;
        }

        cJSON_AddStringToObject(result, "rawResponse", response.c_str());
    }

cleanup:
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        awaiting_response_ = false;
        pending_response_.clear();
        xSemaphoreGive(state_mutex_);
    }
    xSemaphoreGive(command_mutex_);

    if (result == nullptr) {
        result = CreateResultJson("error", "未知错误");
    }
    return result;
}

bool NodeHexaController::SendUartCommand(const std::string& command) {
    const int written = uart_write_bytes(UART_NUM_1, command.c_str(), command.length());
    if (written == static_cast<int>(command.length())) {
        ESP_LOGD(TAG, "UART发送成功: %s", command.c_str());
        return true;
    }

    ESP_LOGE(TAG, "UART发送失败: 期望 %zu 字节, 实际发送 %d 字节", command.length(), written);
    return false;
}

int16_t NodeHexaController::CommandToMovementMode(const std::string& command) {
    if (command == "FORWARD") {
        return 1 << 1;
    } else if (command == "FORWARDFAST") {
        return 1 << 2;
    } else if (command == "BACKWARD") {
        return 1 << 3;
    } else if (command == "TURNLEFT") {
        return 1 << 4;
    } else if (command == "TURNRIGHT") {
        return 1 << 5;
    } else if (command == "SHIFTLEFT") {
        return 1 << 6;
    } else if (command == "SHIFTRIGHT") {
        return 1 << 7;
    } else if (command == "CLIMB") {
        return 1 << 8;
    } else if (command == "ROTATEX") {
        return 1 << 9;
    } else if (command == "ROTATEY") {
        return 1 << 10;
    } else if (command == "ROTATEZ") {
        return 1 << 11;
    } else if (command == "TWIST") {
        return 1 << 12;
    } else if (command == "STANDBY") {
        return 1 << 0;
    }
    return -1;
}

bool NodeHexaController::IsLowBatteryPayload(const cJSON* root) const {
    if (JsonStringEquals(root, "event", "lowBattery")) {
        return true;
    }
    if (JsonStringEquals(root, "code", kLowBatteryProtectCode)) {
        return true;
    }
    return ContainsLowBatteryText(GetJsonString(root, "message"));
}