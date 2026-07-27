#pragma once

#include <cJSON.h>
#include <esp_http_server.h>

class SpippyController;

class SpippyWebServer {
public:
    explicit SpippyWebServer(SpippyController *controller);
    ~SpippyWebServer();
    bool Start();

private:
    esp_err_t StartMdns();
    static esp_err_t HandleRoot(httpd_req_t *req);
    static esp_err_t HandlePage(httpd_req_t *req);
    static esp_err_t HandleStatus(httpd_req_t *req);
    static esp_err_t HandleCalibration(httpd_req_t *req);
    static esp_err_t HandlePreview(httpd_req_t *req);
    static esp_err_t HandlePreflight(httpd_req_t *req);
    static esp_err_t HandleCalibrationPreviewServo(httpd_req_t *req);
    static esp_err_t HandleCalibrationServoUpdate(httpd_req_t *req);
    static esp_err_t HandlePose(httpd_req_t *req);
    static esp_err_t HandleSave(httpd_req_t *req);
    static esp_err_t HandleAutonomy(httpd_req_t *req);

    esp_err_t SendJson(httpd_req_t *req, cJSON *json);
    char *ReadBody(httpd_req_t *req);

    SpippyController *controller_ = nullptr;
    httpd_handle_t server_ = nullptr;
};
