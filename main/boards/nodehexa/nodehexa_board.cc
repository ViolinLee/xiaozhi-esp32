#include <cJSON.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <wifi_manager.h>
#include <ssid_manager.h>
#include <cctype>

#include "application.h"
#include "assets/lang_config.h"
#include "audio/codecs/no_audio_codec.h"
#include "button.h"
#include "config.h"
#include "display.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "nodehexa_controller.h"
#include "system_reset.h"
#include "wifi_board.h"

static const char* kNodeHexaBoardTag = "NodeHexa";

extern void InitializeNodeHexaController();

namespace {

cJSON* CreateBoardError(const char* message) {
    cJSON* result = cJSON_CreateObject();
    cJSON_AddStringToObject(result, "status", "error");
    cJSON_AddStringToObject(result, "message", message);
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
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
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
            const std::string alert_message = message.empty() ? Lang::Strings::BATTERY_NEED_CHARGE : message;
            ESP_LOGW(kNodeHexaBoardTag, "六足进入低电量保护，准备主动提醒: %s", alert_message.c_str());
            auto& app = Application::GetInstance();
            app.Schedule([alert_message]() {
                auto& scheduled_app = Application::GetInstance();
                auto* display = Board::GetInstance().GetDisplay();
                ESP_LOGI(kNodeHexaBoardTag, "<< %s", alert_message.c_str());
                scheduled_app.Alert(
                    Lang::Strings::WARNING,
                    alert_message.c_str(),
                    "triangle_exclamation",
                    Lang::Sounds::OGG_LOW_BATTERY);
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

    std::string GetBoardType() override {
        return "nodehexa";
    }

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
        mcp.AddTool("self.robot.standby", "机器人待机状态。通常在命令停止运动时调用。", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            return nodehexa_controller_->SendCommand("STANDBY");
        });
        
        // 机器人位置控制
        mcp.AddTool("self.robot.position_control", "机器人的位置控制。机器人可以做以下位置控制动作：\n"
            "forward: 前进\nbackward: 后退\nturn_left: 左转\nturn_right: 右转\nshift_left: 左移\nshift_right: 右移\nforward_fast: 快速前进（迈大步）\nclimb: 攀爬（抬高腿）", 
            PropertyList({
                Property("action", kPropertyTypeString),
            }), [this](const PropertyList& properties) -> ReturnValue {
                const std::string& action = properties["action"].value<std::string>();
                std::string command;
                
                if (action == "forward") {
                    command = "FORWARD";
                } else if (action == "backward") {
                    command = "BACKWARD";
                } else if (action == "turn_left") {
                    command = "TURNLEFT";
                } else if (action == "turn_right") {
                    command = "TURNRIGHT";
                } else if (action == "shift_left") {
                    command = "SHIFTLEFT";
                } else if (action == "shift_right") {
                    command = "SHIFTRIGHT";
                } else if (action == "forward_fast") {
                    command = "FORWARDFAST";
                } else if (action == "climb") {
                    command = "CLIMB";
                } else {
                    return CreateBoardError("不支持的位置控制动作");
                }
                
                return nodehexa_controller_->SendCommand(command.c_str());
            });
        
        // 机器人姿态控制
        mcp.AddTool("self.robot.orientation_control", "机器人的姿态控制。机器人可以做以下姿态控制动作：\n"
            "rotate_x: 摇一下头（绕机身X轴旋转）\nrotate_y: 耸一下肩（绕机身Y轴旋转）\nrotate_z: 扭一下身体（绕机身Z轴旋转）\ntwist: 掘屁股、动一下屁股", 
            PropertyList({
                Property("action", kPropertyTypeString),
            }), [this](const PropertyList& properties) -> ReturnValue {
                const std::string& action = properties["action"].value<std::string>();
                std::string command;
                
                if (action == "rotate_x") {
                    command = "ROTATEX";
                } else if (action == "rotate_y") {
                    command = "ROTATEY";
                } else if (action == "rotate_z") {
                    command = "ROTATEZ";
                } else if (action == "twist") {
                    command = "TWIST";
                } else {
                    return CreateBoardError("不支持的姿态控制动作");
                }
                
                return nodehexa_controller_->SendCommand(command.c_str());
            });
        // 机器人速度调节
        mcp.AddTool("self.robot.speed_control", "机器人的速度调节。机器人可以设置以下速度档位：\n"
            "slowest: 极慢速 (0.25倍速)\nslow: 慢速 (0.33倍速)\nmedium: 中速 (0.5倍速，默认)\nfast: 快速 (1.0倍速)\n"
            "参数 speed_level 建议使用 slowest/slow/medium/fast，兼容 0/1/2/3。",
            PropertyList({
                Property("speed_level", kPropertyTypeString),
            }), [this](const PropertyList& properties) -> ReturnValue {
                const std::string& speedLevel = properties["speed_level"].value<std::string>();
                int level = 2;
                if (!ParseSpeedLevel(speedLevel, level)) {
                    ESP_LOGW(kNodeHexaBoardTag, "未识别的 speed_level 参数: %s", speedLevel.c_str());
                    return CreateBoardError("不支持的速度档位");
                }
                
                return nodehexa_controller_->SendSpeedLevelCommand(level);
            });
    }
};

DECLARE_BOARD(NodeHexaBoard);
