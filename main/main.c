#include <stdio.h>
#include <inttypes.h>
#include <sdkconfig.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <string.h>
#include <nvs_flash.h>
#include <nvs.h>

#include "lv_port_disp.h"
#include "camera.h"
#include "gui/main_ui.h"


static const char* TAG = "PocketAI";

static esp_err_t nvs_init() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        return nvs_flash_init();
    }
    return err;
}


static uint32_t custom_tick() {
    return esp_timer_get_time() / 1000;
}

void app_main() {
    lv_port_disp_init();
    lv_tick_set_cb(custom_tick);
    lcd_backlight_on();
    ESP_ERROR_CHECK(nvs_init());

    main_ui_init();

    camera_start();
    ESP_LOGI(TAG, "Init done");

    // ESP_LOGI(TAG, "%lu", clk_ll_cpu_get_freq_mhz_from_pll());

    static uint32_t del_time = 0;
    while (true) {
        main_ui_frame_update();

        del_time = lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(del_time));
    }
}
