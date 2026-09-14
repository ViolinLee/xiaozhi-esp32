#include "nodehexa_controller.h"
#include "nodehexa_actions.h"

#include <cstring>
#include <utility>
#include <vector>

#include <esp_app_desc.h>
#include <esp_timer.h>

namespace {

constexpr EventBits_t kResponseReadyBit = BIT0;
constexpr EventBits_t kCancelledBit = BIT1;
constexpr const char* kLowBatteryProtectCode = "LOW_BATTERY_PROTECT";
constexpr const char* kLowBatteryMessage = "电量低，请关闭电源后进行充电！";
constexpr const char* kCommandBusyMessage = "串口忙，正在等待上一条命令响应";
constexpr const char* kTimeoutMessage = "等待六足主板响应超时";
constexpr uint32_t kUartTaskStackSize = 4096;
constexpr UBaseType_t kUartTaskPriority = 5;
constexpr uint32_t kHeartbeatIntervalMs = 2000;

bool JsonStringEquals(const cJSON* root, const char* key, const char* expected) {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(root, key);
    return cJSON_IsString(item) && item->valuestring != nullptr &&
           strcmp(item->valuestring, expected) == 0;
}

std::string GetJsonString(const cJSON* root, const char* key, const char* fallback = "") {
    const cJSON* item = cJSON_GetObjectItemCaseSensitive(root, key);
    if (cJSON_IsString(item) && item->valuestring != nullptr) {
        return item->valuestring;
    }
    return fallback;
}

bool ContainsLowBatteryText(const std::string& text) {
    return text.find("电量低") != std::string::npos ||
           text.find("low battery") != std::string::npos;
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
    tx_mutex_ = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "NodeHexaController 构造函数");
}

NodeHexaController::~NodeHexaController() {
    if (command_task_)
        vTaskDelete(command_task_);
    if (command_queue_)
        vQueueDelete(command_queue_);
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
    if (tx_mutex_ != nullptr) {
        vSemaphoreDelete(tx_mutex_);
        tx_mutex_ = nullptr;
    }
    ESP_LOGI(TAG, "NodeHexaController 析构函数");
}

void NodeHexaController::Initialize() {
    if (uart_rx_task_handle_ != nullptr) {
        return;
    }
    ESP_LOGI(TAG, "初始化 NodeHexaController");
    if (!command_mutex_ || !state_mutex_ || !tx_mutex_ || !response_event_group_)
        return;
    command_queue_ = xQueueCreate(4, sizeof(CommandJob));
    if (!command_queue_)
        return;
    if (xTaskCreate(NodeHexaController::UartRxTask, "nodehexa_uart_rx", kUartTaskStackSize, this,
                    kUartTaskPriority, &uart_rx_task_handle_) != pdPASS)
        return;
    xTaskCreate(CommandTask, "nodehexa_commands", 6144, this, kUartTaskPriority - 1,
                &command_task_);
}

void NodeHexaController::SetLowBatteryCallback(LowBatteryCallback callback) {
    if (state_mutex_ != nullptr && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
        low_battery_callback_ = std::move(callback);
        xSemaphoreGive(state_mutex_);
    }
}

cJSON* NodeHexaController::SendCommand(const std::string& command, int steps) {
    ESP_LOGI(TAG, "发送命令: %s", command.c_str());

    const std::string mode = nodehexa_actions::Motion(command);
    if (mode.empty()) {
        ESP_LOGE(TAG, "未知命令: %s", command.c_str());
        return CreateResultJson("error", "未知命令");
    }

    cJSON* json_cmd = cJSON_CreateObject();
    if (mode == "standby") {
        // Both generations understand standby; current firmware handles stop first.
        cJSON_AddBoolToObject(json_cmd, "stop", true);
        cJSON_AddNumberToObject(json_cmd, "movementMode", 1);
        return QueueCommand(json_cmd, true);
    }
    if (steps < 1 || steps > 10) {
        cJSON_Delete(json_cmd);
        return CreateResultJson("error", "步数/次数范围为1到10", "INVALID_COUNT");
    }
    cJSON_AddStringToObject(json_cmd, "mode", mode.c_str());
    const bool posture =
        mode == "rotatex" || mode == "rotatey" || mode == "rotatez" || mode == "twist";
    cJSON_AddNumberToObject(json_cmd, posture ? "cycles" : "steps", steps);
    // Do not add legacy movementMode: old firmware must reject unsupported finite actions,
    // never silently turn a finite voice request into unlimited movement.
    return QueueCommand(json_cmd);
}

cJSON* NodeHexaController::SendPerformanceCommand(const std::string& performance) {
    const auto name = nodehexa_actions::Performance(performance);
    if (name.empty())
        return CreateResultJson("error", "不支持的表演动作", "INVALID_ACTION");
    cJSON* command = cJSON_CreateObject();
    cJSON_AddStringToObject(command, "performance", name.c_str());
    cJSON_AddBoolToObject(command, "repeat", false);
    return QueueCommand(command);
}

cJSON* NodeHexaController::QueueCommand(cJSON* command, bool stop) {
    char* payload = cJSON_PrintUnformatted(command);
    cJSON_Delete(command);
    if (!payload || !command_queue_ || !command_task_ ||
        strlen(payload) > nodehexa_uart::kMaxPayloadLength) {
        if (payload)
            cJSON_free(payload);
        return CreateResultJson("error", "控制器不可用", "NOT_READY");
    }
    CommandJob job{};
    strncpy(job.payload, payload, sizeof(job.payload) - 1);
    cJSON_free(payload);
    job.id = ++job_id_;
    if (stop) {
        ++generation_;
        xQueueReset(command_queue_);
        xEventGroupSetBits(response_event_group_, kCancelledBit);
    }
    job.generation = generation_.load();
    if (xQueueSend(command_queue_, &job, 0) != pdTRUE)
        return CreateResultJson("error", "命令队列已满，请稍后再试", "BUSY");
    ESP_LOGI(TAG, "command queued: id=%lu %s", static_cast<unsigned long>(job.id), job.payload);
    cJSON* result =
        CreateResultJson("queued", "已排队发送，尚未确认主板接收或动作完成；可查询status");
    cJSON_AddNumberToObject(result, "requestId", job.id);
    return result;
}

void NodeHexaController::CommandTask(void* arg) {
    static_cast<NodeHexaController*>(arg)->CommandLoop();
}

void NodeHexaController::CommandLoop() {
    NegotiateProtocol();
    uint32_t last_probe = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    while (true) {
        CommandJob job{};
        if (xQueueReceive(command_queue_, &job, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (job.generation != generation_.load())
                continue;
            cJSON* command = cJSON_Parse(job.payload);
            cJSON* result = SendJsonCommandAndWait(command, job.generation);
            cJSON_AddNumberToObject(result, "requestId", job.id);
            char* serialized = cJSON_PrintUnformatted(result);
            ESP_LOGI(TAG, "command result: %s", serialized ? serialized : "allocation failure");
            if (xSemaphoreTake(state_mutex_, portMAX_DELAY) == pdTRUE) {
                last_result_ = serialized ? serialized : "{}";
                xSemaphoreGive(state_mutex_);
            }
            if (serialized)
                cJSON_free(serialized);
            cJSON_Delete(result);
        }
        const uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        // Refresh identity/capabilities after a peer reboot or late power-on. No motion retries.
        if (uxQueueMessagesWaiting(command_queue_) == 0 && now - last_probe >= 5000) {
            NegotiateProtocol();
            last_probe = now;
        }
    }
}

cJSON* NodeHexaController::GetStatus() {
    cJSON* result = CreateResultJson("success", "lastResult为主板接收结果，不代表运动已经完成");
    cJSON_AddStringToObject(
        result, "protocol",
        protocol_mode_.load() == ProtocolMode::V2 ? "v2" : "legacy_or_discovering");
    cJSON_AddNumberToObject(result, "maxSteps", 10);
    cJSON_AddBoolToObject(result, "unlimitedVoiceMotion", false);
    const uint32_t last_response = last_response_ms_.load();
    cJSON_AddBoolToObject(
        result, "recentResponse",
        last_response != 0 &&
            static_cast<uint32_t>(esp_timer_get_time() / 1000) - last_response < 6000);
    cJSON_AddStringToObject(result, "finiteMotionSupport",
                            "以主板命令回执为准；旧主板可能需升级，绝不回退无限运动");
    if (state_mutex_ && xSemaphoreTake(state_mutex_, 0) == pdTRUE) {
        cJSON_AddBoolToObject(result, "lowBattery", low_battery_active_);
        if (!last_result_.empty())
            cJSON_AddItemToObject(result, "lastResult", cJSON_Parse(last_result_.c_str()));
        if (!peer_info_.empty())
            cJSON_AddItemToObject(result, "peer", cJSON_Parse(peer_info_.c_str()));
        xSemaphoreGive(state_mutex_);
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

    return QueueCommand(json_cmd);
}

void NodeHexaController::UartRxTask(void* arg) {
    auto* self = static_cast<NodeHexaController*>(arg);
    self->UartRxLoop();
}

void NodeHexaController::UartRxLoop() {
    while (true) {
        uint8_t bytes[128];
        const int read_len = uart_read_bytes(UART_NUM_1, bytes, sizeof(bytes), pdMS_TO_TICKS(20));
        const uint32_t now_ms = static_cast<uint32_t>(esp_timer_get_time() / 1000);
        if (read_len > 0) {
            nodehexa_uart::Frame frame{};
            for (int i = 0; i < read_len; ++i)
                if (parser_.Feed(bytes[i], now_ms, frame))
                    HandleIncomingFrame(frame);
        } else {
            parser_.PollTimeout(now_ms);
        }
        if (protocol_mode_.load() == ProtocolMode::V2 &&
            now_ms - last_heartbeat_ms_ >= kHeartbeatIntervalMs) {
            last_heartbeat_ms_ = now_ms;
            SendV2Frame(nodehexa_uart::MessageType::Heartbeat, 0, NextSequence(), "{}");
        }
    }
}

void NodeHexaController::HandleIncomingFrame(const nodehexa_uart::Frame& frame) {
    const std::string payload(reinterpret_cast<const char*>(frame.payload), frame.payload_length);
    ESP_LOGD(TAG, "UART接收消息: format=%s type=%u seq=%u",
             frame.format == nodehexa_uart::Format::V2 ? "v2" : "legacy",
             static_cast<unsigned>(frame.message_type), frame.sequence);

    cJSON* root = cJSON_Parse(payload.c_str());
    if (root == nullptr || !cJSON_IsObject(root)) {
        if (root != nullptr) {
            cJSON_Delete(root);
        }
        ESP_LOGW(TAG, "收到无法解析的UART JSON");
        return;
    }

    const bool is_v2 = frame.format == nodehexa_uart::Format::V2;
    if ((is_v2 && frame.message_type == nodehexa_uart::MessageType::Event) ||
        (!is_v2 && cJSON_GetObjectItemCaseSensitive(root, "event") != nullptr)) {
        HandleIncomingEvent(root);
    } else if ((is_v2 && frame.message_type == nodehexa_uart::MessageType::Response) ||
               (!is_v2 && cJSON_GetObjectItemCaseSensitive(root, "status") != nullptr)) {
        HandleIncomingResponse(root, payload, frame.sequence, is_v2);
    } else {
        ESP_LOGW(TAG, "收到未知UART消息类型");
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

void NodeHexaController::HandleIncomingResponse(cJSON* root, const std::string& payload,
                                                uint16_t sequence, bool is_v2) {
    last_response_ms_.store(static_cast<uint32_t>(esp_timer_get_time() / 1000));
    // Heartbeat and HELLO responses carry authoritative power state. This lets
    // a still-running XiaoZhi board re-arm notifications after NodeHexa reboots.
    ClearLowBatteryStateIfExplicitHealthy(root);
    if (IsLowBatteryPayload(root)) {
        const std::string message = GetJsonString(root, "message", kLowBatteryMessage);
        ESP_LOGW(TAG, "收到六足主板低电量响应: %s", message.c_str());
        NotifyLowBattery(message);
    }

    bool delivered = false;
    if (state_mutex_ != nullptr && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (awaiting_response_ && is_v2 == pending_v2_ &&
            (!is_v2 || sequence == pending_sequence_)) {
            pending_response_ = payload;
            awaiting_response_ = false;
            delivered = true;
            // Publish while holding the request lock so a timeout cannot start
            // another transaction between storing the payload and waking it.
            xEventGroupSetBits(response_event_group_, kResponseReadyBit);
        }
        xSemaphoreGive(state_mutex_);
    }

    if (!delivered) {
        ESP_LOGD(TAG, "收到非当前请求响应（包括心跳）: seq=%u", sequence);
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

void NodeHexaController::ClearLowBatteryStateIfExplicitHealthy(const cJSON* root) {
    const cJSON* power = cJSON_GetObjectItemCaseSensitive(root, "power");
    const cJSON* latched = cJSON_IsObject(power)
                               ? cJSON_GetObjectItemCaseSensitive(power, "lowBatteryLatched")
                               : nullptr;
    if (!cJSON_IsFalse(latched))
        return;
    if (state_mutex_ != nullptr && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        low_battery_active_ = false;
        low_battery_notified_ = false;
        xSemaphoreGive(state_mutex_);
    }
}

void NodeHexaController::NegotiateProtocol() {
    if (command_mutex_ == nullptr || state_mutex_ == nullptr || response_event_group_ == nullptr) {
        protocol_mode_.store(ProtocolMode::Legacy);
        return;
    }
    if (xSemaphoreTake(command_mutex_, pdMS_TO_TICKS(HELLO_TIMEOUT_MS)) != pdTRUE) {
        protocol_mode_.store(ProtocolMode::Legacy);
        return;
    }
    const uint16_t sequence = NextSequence();
    xEventGroupClearBits(response_event_group_, kResponseReadyBit | kCancelledBit);
    if (xSemaphoreTake(state_mutex_, portMAX_DELAY) == pdTRUE) {
        awaiting_response_ = true;
        pending_sequence_ = sequence;
        pending_v2_ = true;
        pending_response_.clear();
        xSemaphoreGive(state_mutex_);
    }
    const std::string hello =
        std::string("{\"device\":\"xiaozhi\",\"deviceId\":\"nodehexa-bsp\",\"firmware\":\"") +
        esp_app_get_description()->version + "\",\"protocols\":[2],\"capabilities\":[\"control\"]}";
    bool v2_ready = false;
    if (SendV2Frame(nodehexa_uart::MessageType::Hello, 0x02, sequence, hello)) {
        const EventBits_t bits =
            xEventGroupWaitBits(response_event_group_, kResponseReadyBit | kCancelledBit, pdTRUE,
                                pdFALSE, pdMS_TO_TICKS(HELLO_TIMEOUT_MS));
        if ((bits & kResponseReadyBit) != 0) {
            std::string response;
            if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
                response = pending_response_;
                xSemaphoreGive(state_mutex_);
            }
            cJSON* root = cJSON_Parse(response.c_str());
            if (root != nullptr && JsonStringEquals(root, "device", "nodehexa")) {
                const cJSON* protocol = cJSON_GetObjectItemCaseSensitive(root, "protocol");
                v2_ready = cJSON_IsNumber(protocol) && protocol->valueint == 2;
                if (v2_ready && xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
                    peer_info_ = response;
                    xSemaphoreGive(state_mutex_);
                }
                if (v2_ready)
                    ClearLowBatteryStateIfExplicitHealthy(root);
            }
            if (root != nullptr)
                cJSON_Delete(root);
        }
    }
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        awaiting_response_ = false;
        pending_sequence_ = 0;
        pending_response_.clear();
        xSemaphoreGive(state_mutex_);
    }
    if (!v2_ready && xSemaphoreTake(state_mutex_, portMAX_DELAY) == pdTRUE) {
        peer_info_.clear();
        xSemaphoreGive(state_mutex_);
    }
    protocol_mode_.store(v2_ready ? ProtocolMode::V2 : ProtocolMode::Legacy);
    xSemaphoreGive(command_mutex_);
    ESP_LOGI(TAG, "NodeHexa UART protocol: %s", v2_ready ? "v2" : "legacy fallback");
}

cJSON* NodeHexaController::SendJsonCommandAndWait(cJSON* json_cmd, uint32_t generation) {
    if (command_mutex_ == nullptr || state_mutex_ == nullptr || response_event_group_ == nullptr) {
        cJSON_Delete(json_cmd);
        return CreateResultJson("error", "控制器尚未完成初始化");
    }

    char* json_str = cJSON_PrintUnformatted(json_cmd);
    if (!json_str) {
        cJSON_Delete(json_cmd);
        return CreateResultJson("error", "JSON分配失败", "NO_MEMORY");
    }
    const std::string payload = json_str;
    if (json_str != nullptr) {
        cJSON_free(json_str);
    }
    cJSON_Delete(json_cmd);

    if (xSemaphoreTake(command_mutex_, pdMS_TO_TICKS(UART_TIMEOUT_MS)) != pdTRUE) {
        return CreateResultJson("error", kCommandBusyMessage);
    }

    xEventGroupClearBits(response_event_group_, kResponseReadyBit | kCancelledBit);
    if (generation != generation_.load()) {
        xSemaphoreGive(command_mutex_);
        return CreateResultJson("error", "命令已被停止取消", "CANCELLED");
    }
    const ProtocolMode mode = protocol_mode_.load();
    const uint16_t sequence = mode == ProtocolMode::V2 ? NextSequence() : 0;
    if (xSemaphoreTake(state_mutex_, portMAX_DELAY) == pdTRUE) {
        pending_response_.clear();
        awaiting_response_ = true;
        pending_sequence_ = sequence;
        pending_v2_ = mode == ProtocolMode::V2;
        xSemaphoreGive(state_mutex_);
    }

    cJSON* result = nullptr;
    const bool sent = mode == ProtocolMode::V2 ? SendV2Frame(nodehexa_uart::MessageType::Request,
                                                             0x02, sequence, payload)
                                               : SendLegacyCommand(payload);
    if (!sent) {
        result = CreateResultJson("error", "UART发送失败");
        goto cleanup;
    }

    {
        const EventBits_t bits =
            xEventGroupWaitBits(response_event_group_, kResponseReadyBit | kCancelledBit, pdTRUE,
                                pdFALSE, pdMS_TO_TICKS(UART_TIMEOUT_MS));
        if ((bits & kCancelledBit) != 0) {
            result = CreateResultJson("error", "已中断等待，准备发送停止；原命令是否执行未知",
                                      "CANCELLED");
            goto cleanup;
        }
        if ((bits & kResponseReadyBit) == 0) {
            result = CreateResultJson("error", "主板应答超时，是否执行未知；不要自动重发运动",
                                      "TIMEOUT");
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
            goto cleanup;
        }
    }

cleanup:
    if (xSemaphoreTake(state_mutex_, pdMS_TO_TICKS(20)) == pdTRUE) {
        awaiting_response_ = false;
        pending_sequence_ = 0;
        pending_response_.clear();
        xSemaphoreGive(state_mutex_);
    }
    xSemaphoreGive(command_mutex_);

    if (result == nullptr) {
        result = CreateResultJson("error", "未知错误");
    }
    return result;
}

bool NodeHexaController::SendLegacyCommand(const std::string& payload) {
    const std::string frame = "$" + payload + "\n";
    if (tx_mutex_ == nullptr || xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(100)) != pdTRUE)
        return false;
    const int written = uart_write_bytes(UART_NUM_1, frame.data(), frame.size());
    xSemaphoreGive(tx_mutex_);
    return written == static_cast<int>(frame.size());
}

bool NodeHexaController::SendV2Frame(nodehexa_uart::MessageType type, uint8_t flags,
                                     uint16_t sequence, const std::string& payload) {
    if (payload.size() > nodehexa_uart::kMaxPayloadLength)
        return false;
    std::vector<uint8_t> frame;
    if (!nodehexa_uart::EncodeV2(type, flags, sequence,
                                 reinterpret_cast<const uint8_t*>(payload.data()),
                                 static_cast<uint16_t>(payload.size()), frame)) {
        return false;
    }
    if (tx_mutex_ == nullptr || xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(100)) != pdTRUE)
        return false;
    const int written = uart_write_bytes(UART_NUM_1, frame.data(), frame.size());
    xSemaphoreGive(tx_mutex_);
    return written == static_cast<int>(frame.size());
}

uint16_t NodeHexaController::NextSequence() {
    uint16_t sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    if (sequence == 0)
        sequence = next_sequence_.fetch_add(1, std::memory_order_relaxed);
    return sequence;
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
