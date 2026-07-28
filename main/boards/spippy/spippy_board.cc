#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <wifi_manager.h>
#include <ssid_manager.h>

#include <atomic>
#include <cstdio>

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
        web_server_ = new SpippyWebServer(controller_);
        RegisterNetworkReadyHandler();
    }

    ~SpippyBoard() override {
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, NetworkReadyEventHandler);
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

    void StartNetwork() override {
        WifiBoard::StartNetwork();
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
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = CONFIG_SPIPPY_DISPLAY_OLED_ADDR,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
            .scl_speed_hz = CONFIG_SPIPPY_DISPLAY_I2C_FREQ_HZ,
        };
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c_v2(display_i2c_bus_, &io_config, &panel_io_));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
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
                EnterWifiConfigMode();
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

    void RegisterNetworkReadyHandler() {
        esp_err_t err = esp_event_loop_create_default();
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "failed to create default event loop: %s", esp_err_to_name(err));
            return;
        }
        err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, NetworkReadyEventHandler, this);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to register STA IP handler: %s", esp_err_to_name(err));
        }
    }

    static void NetworkReadyEventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
        (void)event_data;
        auto *self = static_cast<SpippyBoard *>(arg);
        if (self == nullptr) {
            return;
        }
        if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
            ESP_LOGI(TAG, "station connected, starting calibration web server");
            self->StartWebServerLater();
        }
    }

    static void WebServerTaskEntry(void *arg) {
        auto *self = static_cast<SpippyBoard *>(arg);
        vTaskDelay(pdMS_TO_TICKS(1000));
        self->StartWebServer();
        vTaskDelete(nullptr);
    }

    void StartWebServerLater() {
        bool expected = false;
        if (!web_server_task_started_.compare_exchange_strong(expected, true)) {
            return;
        }
        BaseType_t ok = xTaskCreate(WebServerTaskEntry, "spippy_web", 4096, this, 3, nullptr);
        if (ok != pdPASS) {
            web_server_task_started_.store(false);
            ESP_LOGE(TAG, "failed to create web task");
        }
    }

    void StartWebServer() {
        if (web_server_started_.load() || web_server_ == nullptr) {
            return;
        }
        bool started = web_server_->Start();
        web_server_started_.store(started);
        if (!started) {
            web_server_task_started_.store(false);
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
    std::atomic<bool> web_server_task_started_{false};
    std::atomic<bool> web_server_started_{false};
};

DECLARE_BOARD(SpippyBoard);
