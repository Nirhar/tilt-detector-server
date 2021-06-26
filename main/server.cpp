#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <esp_event.h>
#include <esp_system.h>
#include <sys/param.h>
#include <esp_http_server.h>
#include "kalmanfilter.hpp"
#include "esp_eth.h"
#include "protocol_examples_common.h"
#include "esp_tls_crypto.h"

#include <cmath>

const char* TAG = "HTTPServer";
// KALMAN pfilter(0.005);
// KALMAN rfilter(0.005);

httpd_handle_t server;
float ax,ay,az,gx,gy,gz;
float pitch, roll;
float fpitch, froll;

static void mpu6050_task(void *pvParameters) {
    MPU6050 mpu(GPIO_NUM_22, GPIO_NUM_21, I2C_NUM_0);
    if(!mpu.init()) {
	    ESP_LOGE("mpu6050", "init failed!");
        vTaskDelete(0);
    }
	ESP_LOGI("mpu6050", "init success!");
  

    KALMAN pfilter(0.005);
    KALMAN rfilter(0.005);

    uint32_t lasttime = 0;
    int count = 0;

    while(1) {
        ax = -mpu.getAccX();
        ay = -mpu.getAccY();
        az = -mpu.getAccZ();
        gx = mpu.getGyroX();
        gy = mpu.getGyroY();
        gz = mpu.getGyroZ();
        pitch = atan(ax/az)*57.2958;
        roll = atan(ay/az)*57.2958;
        fpitch = pfilter.filter(pitch, gy);
        froll = rfilter.filter(roll, -gx);
        count++;
        if(esp_log_timestamp() / 1000 != lasttime) {
            lasttime = esp_log_timestamp() / 1000;
            // printf("-------------------------------------------------------\n");
            // printf("mpu6050 Samples: %d\n", count);
            // count = 0;
            // printf("mpu6050 Acc: ( %.3f, %.3f, %.3f)\n", ax, ay, az);
            // printf("mpu6050 Gyro: ( %.3f, %.3f, %.3f)\n", gx, gy, gz);
            // printf("mpu6050 Pitch: %.3f\n", pitch);
            // printf("mpu6050 Roll: %.3f\n", roll);
            // printf("mpu6050 FPitch: %.3f\n", fpitch);
            // printf("mpu6050 FRoll: %.3f\n", froll);
        }
    }

}

void setup_wifi(){

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_scan_threshold_t thresh = {
        .authmode = WIFI_AUTH_WPA2_PSK,
    };
    wifi_config_t wifi_config = {
        .sta = {
            {.ssid = "HARI"},
            {.password = "vu2hkevu2hkevu2hke"},
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold = thresh,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
}

/* URI handler structure for GET /uri */
esp_err_t get_hello_handler(httpd_req_t *req)
{
    /* Send a simple response */
    const char resp[] = "Hi from Esp-Server!\n";
    printf("Hello URL accessed!\n");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
httpd_uri_t get_hello = {
    .uri      = "/hello",
    .method   = HTTP_GET,
    .handler  = get_hello_handler,
    .user_ctx = NULL
};

/* URI handler structure for GET /uri */
esp_err_t get_rp_handler(httpd_req_t *req)
{
    /* Send a simple response */
    
    
    printf("Angle URL accessed\n");
    char Buffer[35];
    sprintf(Buffer,"{\"roll\":%.3f , \"pitch\":%.3f}",roll,pitch);
    httpd_resp_send(req, Buffer,HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
httpd_uri_t get_rp = {
    .uri      = "/get_angles",
    .method   = HTTP_GET,
    .handler  = get_rp_handler,
    .user_ctx = NULL
};



static httpd_handle_t start_webserver(void)
{
    // httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &get_hello);
        httpd_register_uri_handler(server, &get_rp);
        
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

static void start_webserver_task(void* param){
    server = start_webserver();
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}

extern "C" void app_main(void)
{   
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    // setup_wifi();
    ESP_ERROR_CHECK(example_connect());

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
	xTaskCreatePinnedToCore(&mpu6050_task,"mpu6050_task",2048,NULL,5,NULL,0);
    // xTaskCreatePinnedToCore(&start_webserver_task,"server_task",4096,NULL,5,NULL,1);

    // MPU6050 mpu(GPIO_NUM_22, GPIO_NUM_21, I2C_NUM_0);
    // g_mpu = &mpu;

    // esp_netif_ip_info_t ip_info;
    // esp_netif_get_ip_info(ESP_IF_WIFI_STA,&ip_info);
    // ESP_LOGI(TAG,"My IP: " IPSTR "\n", IP2STR(&ip_info.ip));

    // ESP_LOGI(TAG,"IP Address is %s PORT is 80\n",ip_addr);

    /* Start the server for the first time */
    server = start_webserver();
}


