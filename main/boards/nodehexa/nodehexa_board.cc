#include <driver/uart.h>
#include <esp_log.h>
#include <cJSON.h>
#include <ssid_manager.h>
#include <wifi_manager.h>
#include <cctype>
#include <stdexcept>

#include "application.h"
#include "assets/lang_config.h"
#include "audio/codecs/no_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "nodehexa_actions.h"
#include "nodehexa_controller.h"
#include "system_reset.h"
#include "wifi_board.h"

static const char* kNodeHexaBoardTag = "NodeHexa";

extern void InitializeNodeHexaController();

namespace {

cJSON* CreateBoardError(const char* message) { throw std::invalid_argument(message); }

cJSON* CheckedResult(cJSON* result) {
    if (!result)
        throw std::runtime_error("控制器结果分配失败");
    const cJSON* status = cJSON_GetObjectItemCaseSensitive(result, "status");
    if (cJSON_IsString(status) && std::string(status->valuestring) == "error") {
        const cJSON* message = cJSON_GetObjectItemCaseSensitive(result, "message");
        const std::string text = cJSON_IsString(message) ? message->valuestring : "控制失败";
        cJSON_Delete(result);
        throw std::runtime_error(text);
    }
    return result;
}

}  // namespace

class NodeHexaBoard : public WifiBoard {
private:
    Button boot_button_;
    NodeHexaController* nodehexa_controller_;

    static std::string NormalizeSpeedLevel(std::string speed_level) {
        std::string normalized;
        normalized.reserve(speed_level.size());
        for (unsigned char c : speed_level) {
            if (c == ' ' || c == '_' || c == '-') {
                continue;
            }
            if (c < 128) {
                normalized.push_back(static_cast<char>(std::tolower(c)));
            } else {
                normalized.push_back(static_cast<char>(c));
            }
        }
        return normalized;
    }

    static bool ParseSpeedLevel(const std::string& speed_level, int& level) {
        std::string token = NormalizeSpeedLevel(speed_level);
        if (token.empty()) {
            return false;
        }

        if (token == "0" || token == "slowest" || token == "veryslow" || token == "ultraslow" ||
            token == "最慢" || token == "极慢" || token == "超慢") {
            level = 0;
            return true;
        }
        if (token == "1" || token == "slow" || token == "slower" || token == "稍慢" ||
            token == "慢" || token == "慢一点" || token == "减速") {
            level = 1;
            return true;
        }
        if (token == "2" || token == "medium" || token == "normal" || token == "default" ||
            token == "中速" || token == "正常速度" || token == "标准速度") {
            level = 2;
            return true;
        }
        if (token == "3" || token == "fast" || token == "faster" || token == "fastest" ||
            token == "quick" || token == "快" || token == "快一点" || token == "加速" ||
            token == "最快") {
            level = 3;
            return true;
        }

        return false;
    }

    void InitializeUart() {
        // 初始化UART1用于与六足机器人通信 (ESP32-S3默认引脚: GPIO17-TX, GPIO18-RX)
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024, 1024, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE,
                                     UART_PIN_NO_CHANGE));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting &&
                !WifiManager::GetInstance().IsConnected()) {
                // WiFi connect component API no longer provides WifiStation singleton.
                // Clear stored SSID list then enter config mode.
                SsidManager::GetInstance().Clear();
                EnterWifiConfigMode();
            }
            app.ToggleChatState();
        });
    }

    void InitializeNodeHexaController() {
        nodehexa_controller_ = new NodeHexaController();
        nodehexa_controller_->SetLowBatteryCallback([](const std::string& message) {
            const std::string alert_message =
                message.empty() ? Lang::Strings::BATTERY_NEED_CHARGE : message;
            ESP_LOGW(kNodeHexaBoardTag, "六足进入低电量保护，准备主动提醒: %s",
                     alert_message.c_str());
            auto& app = Application::GetInstance();
            app.Schedule([alert_message]() {
                auto& scheduled_app = Application::GetInstance();
                auto* display = Board::GetInstance().GetDisplay();
                ESP_LOGI(kNodeHexaBoardTag, "<< %s", alert_message.c_str());
                scheduled_app.Alert(Lang::Strings::WARNING, alert_message.c_str(),
                                    "triangle_exclamation", Lang::Sounds::OGG_LOW_BATTERY);
                if (display != nullptr) {
                    display->SetChatMessage("assistant", alert_message.c_str());
                }
            });
        });
        nodehexa_controller_->Initialize();
    }

public:
    NodeHexaBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        ESP_LOGI(kNodeHexaBoardTag, "初始化 NodeHexa 六足机器人主板");

        InitializeUart();
        InitializeButtons();
        InitializeNodeHexaController();
        InitializeTools();
    }

    ~NodeHexaBoard() {
        if (nodehexa_controller_) {
            delete nodehexa_controller_;
        }
    }

    std::string GetBoardType() override { return "nodehexa"; }

    AudioCodec* GetAudioCodec() override {
        // 右声道配置
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                               AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
                                               AUDIO_I2S_SPK_GPIO_DOUT, I2S_STD_SLOT_RIGHT,
                                               AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS,
                                               AUDIO_I2S_MIC_GPIO_DIN, I2S_STD_SLOT_LEFT);

        // 双声道配置（如果需要同时输出左右声道）
        // static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
        //                                        AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
        //                                        AUDIO_I2S_SPK_GPIO_DOUT, I2S_STD_SLOT_BOTH,
        //                                        AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS,
        //                                        AUDIO_I2S_MIC_GPIO_DIN, I2S_STD_SLOT_LEFT);

        return &audio_codec;
    }

    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();

        // 机器人待机状态
        mcp.AddTool("self.robot.standby",
                    "停止机器人运动和表演。听到停止、停下、别动、待机时调用，无需参数。",
                    PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                        return CheckedResult(nodehexa_controller_->SendCommand("STANDBY"));
                    });

        // 机器人位置控制
        mcp.AddTool(
            "self.robot.position_control",
            "让六足机器人实际行走或转向。用户说前进、后退、左转等时调用本工具。"
            "action: "
            "forward/前进、backward/后退、turn_left/左转、turn_right/右转、shift_left/"
            "左移、shift_right/右移、forward_fast/迈大步、climb/攀爬。"
            "steps为步数，默认3，范围1到10，走完自动停止；前进五步用action=forward,steps="
            "5。未指定步数用默认值。"
            "不支持无限持续或精确厘米/"
            "角度，不得把一直走解释为无限运动。queued仅表示已排队，可用status查询主板回执。",
            PropertyList({
                Property("action", kPropertyTypeString),
                Property("steps", kPropertyTypeInteger, 3, 1, 10),
            }),
            [this](const PropertyList& properties) -> ReturnValue {
                const std::string action =
                    nodehexa_actions::Motion(properties["action"].value<std::string>());
                std::string command;

                if (action == "forward") {
                    command = "FORWARD";
                } else if (action == "backward") {
                    command = "BACKWARD";
                } else if (action == "turnleft") {
                    command = "TURNLEFT";
                } else if (action == "turnright") {
                    command = "TURNRIGHT";
                } else if (action == "shiftleft") {
                    command = "SHIFTLEFT";
                } else if (action == "shiftright") {
                    command = "SHIFTRIGHT";
                } else if (action == "forwardfast") {
                    command = "FORWARDFAST";
                } else if (action == "climb") {
                    command = "CLIMB";
                } else if (action == "standby") {
                    command = "STANDBY";
                } else {
                    return CreateBoardError("不支持的位置控制动作");
                }

                return CheckedResult(
                    nodehexa_controller_->SendCommand(command, properties["steps"].value<int>()));
            });

        // 机器人姿态控制
        mcp.AddTool("self.robot.orientation_control",
                    "机器人的姿态控制。机器人可以做以下姿态控制动作：\n"
                    "rotate_x/摇头、rotate_y/耸肩、rotate_z/扭身体、twist/扭屁股。"
                    "cycles为完整动作次数，默认1，范围1到10，做完自动停止。",
                    PropertyList({
                        Property("action", kPropertyTypeString),
                        Property("cycles", kPropertyTypeInteger, 1, 1, 10),
                    }),
                    [this](const PropertyList& properties) -> ReturnValue {
                        const std::string action =
                            nodehexa_actions::Motion(properties["action"].value<std::string>());
                        std::string command;

                        if (action == "rotatex") {
                            command = "ROTATEX";
                        } else if (action == "rotatey") {
                            command = "ROTATEY";
                        } else if (action == "rotatez") {
                            command = "ROTATEZ";
                        } else if (action == "twist") {
                            command = "TWIST";
                        } else {
                            return CreateBoardError("不支持的姿态控制动作");
                        }

                        return CheckedResult(nodehexa_controller_->SendCommand(
                            command, properties["cycles"].value<int>()));
                    });
        mcp.AddTool(
            "self.robot.performance",
            "让六足机器人表演一轮，结束自动停止。"
            "action: "
            "showtime/登场秀、freestyle/自由舞/跳舞、beatsway/律动。说表演登场秀时用showtime。",
            PropertyList({Property("action", kPropertyTypeString)}),
            [this](const PropertyList& properties) -> ReturnValue {
                return CheckedResult(nodehexa_controller_->SendPerformanceCommand(
                    properties["action"].value<std::string>()));
            });
        mcp.AddTool(
            "self.robot.status",
            "查询六足主板协议、能力声明、低电量状态及最近命令回执。"
            "queued不代表执行成功；lastResult是接收回执，不是运动完成通知。不要在每次前进前查询。",
            PropertyList(), [this](const PropertyList&) -> ReturnValue {
                return nodehexa_controller_->GetStatus();
            });
        // 机器人速度调节
        mcp.AddTool("self.robot.speed_control",
                    "机器人的速度调节。机器人可以设置以下速度档位：\n"
                    "slowest: 极慢速 (0.25倍速)\nslow: 慢速 (0.33倍速)\nmedium: 中速 "
                    "(0.5倍速，默认)\nfast: 快速 (1.0倍速)\n"
                    "参数 speed_level 建议使用 slowest/slow/medium/fast，兼容 0/1/2/3。",
                    PropertyList({
                        Property("speed_level", kPropertyTypeString),
                    }),
                    [this](const PropertyList& properties) -> ReturnValue {
                        const std::string& speedLevel =
                            properties["speed_level"].value<std::string>();
                        int level = 2;
                        if (!ParseSpeedLevel(speedLevel, level)) {
                            ESP_LOGW(kNodeHexaBoardTag, "未识别的 speed_level 参数: %s",
                                     speedLevel.c_str());
                            return CreateBoardError("不支持的速度档位");
                        }

                        return CheckedResult(nodehexa_controller_->SendSpeedLevelCommand(level));
                    });
    }
};

DECLARE_BOARD(NodeHexaBoard);
