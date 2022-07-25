#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"

#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

static const char* TAG = "example";

static constexpr gpio_num_t SDA = GPIO_NUM_22;
static constexpr gpio_num_t SCL = GPIO_NUM_23;
static constexpr uint32_t CLOCK_SPEED = 400000;  // range from 100 KHz ~ 400Hz

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch(event_id){
        case WIFI_EVENT_STA_START:
            printf("Conectando...\n");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            printf("Conectado.\n");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            printf("Conexao perdida.\n");
            printf("Tentando reconectar...\n");
            esp_wifi_connect();
            break;
        case IP_EVENT_STA_GOT_IP:
            printf("IP encontrado.\n");
            break;
        default:
            break;
     }
 }

extern "C" void app_main() {

    nvs_flash_init();
    //Wifi/LwIP init
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);

    // Wifi configuration
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration ={
        .sta ={
            .ssid = "S20FE",
            .password = "yskg3597"
        }
    };
    esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration);

    // Wifi start
    esp_wifi_start();

    // Wifi connect
    esp_wifi_connect();

    printf("$ MPU Driver Example: MPU-I2C\n");
    fflush(stdout);

    // Initialize I2C on port 0 using I2Cbus interface
    i2c0.begin(SDA, SCL, CLOCK_SPEED);

    MPU_t MPU;  // create a default MPU object
    MPU.setBus(i2c0);  // set bus port, not really needed since default is i2c0
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  // set address, default is AD0_LOW

    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize());  // initialize the chip and set initial configurations
    // Calibrate
    mpud::raw_axes_t accelBias, gyroBias;
    ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
    ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));
    // Reading sensor data
    printf("Reading sensor data:\n");
    mpud::raw_axes_t accelRaw;   // x, y, z axes as int16
    mpud::raw_axes_t gyroRaw;    // x, y, z axes as int16
    mpud::float_axes_t accelG;   // accel axes in (g) gravity format
    mpud::float_axes_t gyroDPS;  // gyro axes in (DPS) º/s format
    while (true) {
        
        // Read
        MPU.acceleration(&accelRaw);  // fetch raw data from the registers
        MPU.rotation(&gyroRaw);       // fetch raw data from the registers
        // MPU.motion(&accelRaw, &gyroRaw);  // read both in one shot
        // Convert
        accelG = mpud::accelGravity(accelRaw, mpud::ACCEL_FS_4G);
        gyroDPS = mpud::gyroDegPerSec(gyroRaw, mpud::GYRO_FS_500DPS);
        // Debug
        printf("accel: [%+6.2f %+6.2f %+6.2f ] (G) \t", accelG.x, accelG.y, accelG.z);
        printf("gyro: [%+7.2f %+7.2f %+7.2f ] (º/s)\n", gyroDPS[0], gyroDPS[1], gyroDPS[2]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
