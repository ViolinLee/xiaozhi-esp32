#include <esp_event.h>
#include <esp_lcd_io_i2c.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <ssid_manager.h>
#include <wifi_manager.h>

#include <atomic>
#include <cstdio>
#include <utility>

#include "application.h"
#include "assets/lang_config.h"
#include "audio/codecs/no_audio_codec.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "spippy_controller.h"
#include "spippy_display.h"
#include "spippy_web_server.h"
#include "wifi_board.h"

#include <driver/i2c_master.h>

static const char *TAG = "SpippyBoard";

class SpippyBoard : public WifiBoard {
public:
    SpippyBoard() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        controller_ = new SpippyController(spippy_display_);
        controller_->Initialize();
        controller_->RegisterMcpTools();
        InitializeTools();
        web_server_ = new SpippyWebServer(controller_);
        web_task_stopped_ = xSemaphoreCreateBinary();
        if (web_task_stopped_ == nullptr ||
            xTaskCreate(WebServerTaskEntry, "spippy_web", 4096, this, 3, &web_task_) != pdPASS) {
            ESP_LOGE(TAG, "failed to create web lifecycle task");
        }
    }

    ~SpippyBoard() override {
        WifiBoard::SetNetworkEventCallback(nullptr);
        if (web_task_ != nullptr) {
            xTaskNotify(web_task_, kShutdownWebTask, eSetBits);
            xSemaphoreTake(web_task_stopped_, portMAX_DELAY);
        }
        if (web_task_stopped_ != nullptr) {
            vSemaphoreDelete(web_task_stopped_);
        }
        delete web_server_;
        delete controller_;
        delete display_;
        if (display_i2c_bus_ != nullptr) {
            i2c_del_master_bus(display_i2c_bus_);
            display_i2c_bus_ = nullptr;
        }
    }

    std::string GetBoardType() override {
        return "spippy";
    }

    AudioCodec *GetAudioCodec() override {
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                               AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
                                               AUDIO_I2S_SPK_GPIO_DOUT, I2S_STD_SLOT_RIGHT,
                                               AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS,
                                               AUDIO_I2S_MIC_GPIO_DIN, I2S_STD_SLOT_LEFT);
        return &audio_codec;
    }

    Display *GetDisplay() override {
        return display_;
    }

    void SetNetworkEventCallback(NetworkEventCallback callback) override {
        WifiBoard::SetNetworkEventCallback(
            [this, callback = std::move(callback)](NetworkEvent event, const std::string& data) {
                if (event == NetworkEvent::WifiConfigModeExit) {
                    wifi_config_pending_.store(false);
                } else if (event == NetworkEvent::Connected) {
                    NotifyWebTask(kStartWeb);
                } else if (event == NetworkEvent::Disconnected) {
                    NotifyWebTask(kStopWeb);
                }
                if (callback) {
                    callback(event, data);
                }
            });
    }

    bool GetBatteryLevel(int &level, bool &charging, bool &discharging) override {
        return controller_ != nullptr && controller_->GetBatteryLevel(level, charging, discharging);
    }

    std::string GetDeviceStatusJson() override {
        std::string base = WifiBoard::GetDeviceStatusJson();
        cJSON *root = cJSON_Parse(base.c_str());
        if (root == nullptr) {
            return base;
        }
        cJSON_AddStringToObject(root, "product_name", "Spippy");
        cJSON_AddStringToObject(root, "product_name_zh", "斯皮皮");
        cJSON_AddItemToObject(root, "robot", controller_->StatusJson());
        char *text = cJSON_PrintUnformatted(root);
        if (text == nullptr) {
            cJSON_Delete(root);
            ESP_LOGE(TAG, "failed to serialize device status");
            return base;
        }
        std::string result(text);
        cJSON_free(text);
        cJSON_Delete(root);
        return result;
    }

private:
    void InitializeTools() {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.system.reconfigure_wifi",
                    "重新配网、更换 WiFi：结束当前对话并进入 WiFi 配网模式。"
                    "调用前必须向用户确认，因为当前网络连接和语音对话将会断开。",
                    PropertyList(), [this](const PropertyList&) -> ToolResult {
                        if (!RequestWifiConfigMode()) {
                            return std::unexpected("当前状态无法进入配网，请稍后再试");
                        }
                        return "已请求进入配网模式";
                    });
    }

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = static_cast<i2c_port_t>(CONFIG_SPIPPY_DISPLAY_I2C_PORT),
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSsd1306Display() {
        esp_lcd_panel_io_i2c_config_t io_config = {};
        io_config.dev_addr = CONFIG_SPIPPY_DISPLAY_OLED_ADDR;
        io_config.scl_speed_hz = CONFIG_SPIPPY_DISPLAY_I2C_FREQ_HZ;
        io_config.control_phase_bytes = 1;
        io_config.dc_bit_offset = 6;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "display init failed");
            /*
             * 不能把空 panel 交给 OledDisplay：esp_lvgl_port 内部会直接断言。
             * OLED 故障时退化为无屏模式，机器人运动与语音仍可继续工作。
             */
            esp_lcd_panel_del(panel_);
            panel_ = nullptr;
            esp_lcd_panel_io_del(panel_io_);
            panel_io_ = nullptr;
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));
        spippy_display_ = new SpippyDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = spippy_display_;
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto &app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiManager::GetInstance().IsConnected()) {
                SsidManager::GetInstance().Clear();
                RequestWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        boot_button_.OnLongPress([this]() {
            if (spippy_display_ != nullptr) {
                spippy_display_->ShowWebConsoleAddress(CurrentWebUrl());
            }
        });
    }

    std::string CurrentWebUrl() const {
        esp_netif_ip_info_t ip_info = {};
        esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif != nullptr && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            char url[64];
            snprintf(url, sizeof(url), "http://" IPSTR "/spippy", IP2STR(&ip_info.ip));
            return url;
        }
        // 配网 AP 的 80 端口由 WiFiConfigurationAp 的配网页面独占，不能指向 Spippy 页面。
        netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
        if (netif != nullptr && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
            char url[48];
            snprintf(url, sizeof(url), "http://" IPSTR, IP2STR(&ip_info.ip));
            return url;
        }
        return "http://spippy.local/spippy";
    }

    static constexpr uint32_t kStartWeb = 1;
    static constexpr uint32_t kStopWeb = 2;
    static constexpr uint32_t kConfigureWifi = 4;
    static constexpr uint32_t kShutdownWebTask = 8;

    void NotifyWebTask(uint32_t events) {
        if (web_task_ != nullptr) {
            xTaskNotify(web_task_, events, eSetBits);
        }
    }

    bool CanEnterWifiConfigMode() const {
        const auto state = Application::GetInstance().GetDeviceState();
        return state == kDeviceStateStarting || state == kDeviceStateIdle ||
               state == kDeviceStateListening || state == kDeviceStateSpeaking ||
               state == kDeviceStateNotifying;
    }

    bool RequestWifiConfigMode() {
        if (web_task_ == nullptr) {
            return false;
        }
        if (wifi_config_pending_.load()) {
            return true;
        }
        if (!CanEnterWifiConfigMode()) {
            return false;
        }
        if (!wifi_config_pending_.exchange(true)) {
            NotifyWebTask(kConfigureWifi);
        }
        return true;
    }

    static void WebServerTaskEntry(void* arg) {
        auto* self = static_cast<SpippyBoard*>(arg);
        self->RunWebServerTask();
        xSemaphoreGive(self->web_task_stopped_);
        vTaskDelete(nullptr);
    }

    void RunWebServerTask() {
        uint32_t events = 0;
        while (xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY) == pdTRUE) {
            if (events & kShutdownWebTask) {
                web_server_->Stop();
                return;
            }
            if (events & kConfigureWifi) {
                // httpd_stop waits for active handlers. Keep it off the application
                // and WiFi event tasks, and release both ports before starting the AP.
                if (!web_server_->Stop()) {
                    wifi_config_pending_.store(false);
                    ESP_LOGE(TAG, "cannot release calibration web ports for provisioning");
                    continue;
                }
                Application::GetInstance().Schedule([this]() {
                    if (CanEnterWifiConfigMode()) {
                        WifiBoard::EnterWifiConfigMode();
                    } else {
                        wifi_config_pending_.store(false);
                        NotifyWebTask(kStartWeb);
                        ESP_LOGW(TAG, "device state changed before provisioning");
                    }
                });
                continue;
            }
            if (events & kStopWeb) {
                web_server_->Stop();
            }
            // Ignore stale connect events and AP-mode credential test connections.
            // All HTTP start/stop calls run on this task, including a start already
            // in progress when a provisioning request arrives.
            if ((events & kStartWeb) && !wifi_config_pending_.load() &&
                !WifiManager::GetInstance().IsConfigMode() &&
                WifiManager::GetInstance().IsConnected()) {
                web_server_->Start();
            }
        }
    }

    i2c_master_bus_handle_t display_i2c_bus_ = nullptr;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display *display_ = nullptr;
    SpippyDisplay *spippy_display_ = nullptr;
    Button boot_button_;
    SpippyController *controller_ = nullptr;
    SpippyWebServer *web_server_ = nullptr;
    TaskHandle_t web_task_ = nullptr;
    SemaphoreHandle_t web_task_stopped_ = nullptr;
    std::atomic<bool> wifi_config_pending_{false};
};

DECLARE_BOARD(SpippyBoard);
