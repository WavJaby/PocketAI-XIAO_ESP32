//
// Created by user on 2025/4/11.
//

#include "camera.h"

#include <esp_log.h>
#include <esp_err.h>
#include <JPEGDEC.h>
#include <espressif__esp32-camera/target/private_include/ll_cam.h>
#include <stdbool.h>

#include "atomic.h"

#ifdef CONFIG_SCCB_HARDWARE_I2C_PORT0
#define CAM_I2C_PORT 0
#elifdef CONFIG_SCCB_HARDWARE_I2C_PORT1
#define CAM_I2C_PORT 1
#endif

#define CAM_FRAME_READY 0
#define CAM_FRAME_AVAILABLE 1

#define CAM_PIN_PWDN    -1  //power down is not used
#define CAM_PIN_RESET   -1  //software reset will be performed
#define CAM_PIN_XCLK    10  // XMCLK
#define CAM_PIN_SDA     40  // CAM_SDA
#define CAM_PIN_SCL     39  // CAM_SCL

#define CAM_PIN_D0      15  // DVP_Y2
#define CAM_PIN_D1      17  // DVP_Y3
#define CAM_PIN_D2      18  // DVP_Y4
#define CAM_PIN_D3      16  // DVP_Y5
#define CAM_PIN_D4      14  // DVP_Y6
#define CAM_PIN_D5      12  // DVP_Y7
#define CAM_PIN_D6      11  // DVP_Y8
#define CAM_PIN_D7      48  // DVP_Y9

#define CAM_PIN_VSYNC   38  // DVP_VSYNC
#define CAM_PIN_HREF    47  // DVP_HREF
#define CAM_PIN_PCLK    13  // DVP_PCLK

#define CAM_MAX_FRAME_BUFFER_COUNT 3
#define CAM_MAX_FRAMERATE 30

#define CAM_PREVIEW_QUALITY 4
#define CAM_PREVIEW_SIZE FRAMESIZE_HQVGA
#define CAM_PREVIEW_SCALE 0

// #define CAM_PREVIEW_QUALITY 8
// #define CAM_PREVIEW_SIZE FRAMESIZE_HVGA
// #define CAM_PREVIEW_SCALE JPEG_SCALE_HALF

#define CAM_CAPTURE_QUALITY 2
#define CAM_CAPTURE_SIZE FRAMESIZE_UXGA
#define CAM_CAPTURE_SCALE JPEG_SCALE_EIGHTH


static const char* TAG = "camera";

extern cam_obj_t* cam_obj;
extern int cam_verify_jpeg_eoi(const uint8_t* inbuf, uint32_t length);

camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SDA,
    .pin_sccb_scl = CAM_PIN_SCL,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = CAM_PREVIEW_SIZE,
    // .frame_size = FRAMESIZE_QVGA,
    // .frame_size = FRAMESIZE_SVGA,
    // .frame_size = FRAMESIZE_UXGA,

    .jpeg_quality = CAM_PREVIEW_QUALITY,
    .fb_count = CAM_MAX_FRAME_BUFFER_COUNT,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .sccb_i2c_port = CAM_I2C_PORT,
    .xclk_freq_hz = 20 * 1000 * 1000,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

SemaphoreHandle_t preview_ready_semaphore = NULL;
// SemaphoreHandle_t preview_update_semaphore = NULL;
uint32_t preview_img_width = 0, preview_img_height = 0;

typedef enum {
    PREVIEW,
    CAPTURE,
} camera_state;

static AtomicInt shared_camera_state = atomic_int_init(PREVIEW);

static int jpg_scale = CAM_PREVIEW_SCALE;

static SemaphoreHandle_t convert_pause = NULL;
static QueueHandle_t camera_resolution_change = NULL;
static QueueHandle_t camera_frame_queue = NULL, camera_frame_free_queue = NULL;

static AtomicPointer shared_capture_callback = atomic_ptr_init(NULL);

/*
#define container_of(ptr, type, member)                             \
    __extension__({                                                 \
        const __typeof__(((type *) 0)->member) *(__pmember) = (ptr);\
        (type *) ((char *) __pmember - offsetof(type, member));     \
    })

void my_cam_give(const camera_fb_t* fb_ptr) {
    if (!cam_obj) return;
    cam_frame_t* frame = container_of(fb_ptr, cam_frame_t, fb);
    frame->en = CAM_FRAME_AVAILABLE;
}
*/

static bool cam_check_capture(int* frame_pos) {
    const capture_callback callback = atomic_ptr_get(&shared_capture_callback);
    if (callback) {
        cam_obj->frames[*frame_pos].en = CAM_FRAME_AVAILABLE;
        camera_state state = (camera_state)atomic_int_get(&shared_camera_state);
        switch (state) {
        case PREVIEW:
            camera_set_resolution(CAM_CAPTURE_QUALITY, CAM_CAPTURE_SIZE);
            atomic_int_set(&shared_camera_state, CAPTURE);
            break;
        case CAPTURE:
            callback(&cam_obj->frames[*frame_pos].fb);
            atomic_ptr_set(&shared_capture_callback, NULL);
            camera_set_resolution(CAM_PREVIEW_QUALITY, CAM_PREVIEW_SIZE);
            atomic_int_set(&shared_camera_state, PREVIEW);
            break;
        }
        return true;
    }
    return false;
}

static bool cam_check_resolution_change(int* frame_pos) {
    static int camera_frame_skip = 0;

    if (!camera_frame_skip) {
        xQueuePeek(camera_resolution_change, &camera_frame_skip, 0);
        // Pause convertion
        if (camera_frame_skip) {
            xSemaphoreGive(convert_pause);
        }
    }

    if (camera_frame_skip) {
        // Check frame skip end
        if (--camera_frame_skip == 0) {
            xQueueReset(camera_resolution_change);
        }

        cam_obj->frames[*frame_pos].en = CAM_FRAME_AVAILABLE;

        // Free all preview frame
        while (xQueueReceive(camera_frame_queue, frame_pos, 0) == pdTRUE) {
            cam_obj->frames[*frame_pos].en = CAM_FRAME_AVAILABLE;
        }

        if (xQueueReceive(camera_frame_free_queue, frame_pos, 0) == pdTRUE) {
            cam_obj->frames[*frame_pos].en = CAM_FRAME_AVAILABLE;
        }
        return true;
    }
    return false;
}

extern bool cam_get_next_frame(int* frame_pos) {
    if (!cam_obj) {
        ESP_LOGE(TAG, "cam_obj is NULL!");
        return false;
    }

    if (cam_obj->state == CAM_STATE_IDLE) {
        // Skip frame if resolution change
        if (cam_check_resolution_change(frame_pos))
            return false;

        if (cam_obj->frames[*frame_pos].en == CAM_FRAME_AVAILABLE)
            return true;

        // Find free frame in queue
        if (xQueueReceive(camera_frame_free_queue, frame_pos, 0) == pdTRUE) {
            cam_obj->frames[*frame_pos].en = CAM_FRAME_AVAILABLE;
            return true;
        }

        return false;
    }

    if (cam_obj->frames[*frame_pos].en == CAM_FRAME_READY) {
        camera_fb_t* fb;
        xQueueReceive(cam_obj->frame_buffer_queue, &fb, 0);

        // Skip frame if resolution change
        if (cam_check_resolution_change(frame_pos))
            return false;

        if (!fb) {
            cam_obj->frames[*frame_pos].en = CAM_FRAME_AVAILABLE;
            return true;
        }

        const sensor_t* sensor = esp_camera_sensor_get();
        if (!sensor) {
            ESP_LOGE(TAG, "sensor is null!");
            return false;
        }

        fb->width = resolution[sensor->status.framesize].width;
        fb->height = resolution[sensor->status.framesize].height;
        fb->format = sensor->pixformat;

        int next_frame_pos = -1;
        // Find free frame in queue
        if (xQueueReceive(camera_frame_free_queue, &next_frame_pos, 0) == pdTRUE) {
            cam_obj->frames[next_frame_pos].en = CAM_FRAME_AVAILABLE;
        }

        if (cam_check_capture(frame_pos))
            return false;

        // Send frame, skip if full
        if (xQueueSend(camera_frame_queue, frame_pos, 0) == pdFAIL) {
            ESP_LOGE(TAG, "failed to send frame to queue");
            return false;
        }

        // Find available frame from queue
        if (next_frame_pos != -1) {
            *frame_pos = next_frame_pos;
            return true;
        }

        // Buffer have available frame
        next_frame_pos = (*frame_pos + 1) % (int)cam_obj->frame_cnt;
        if (cam_obj->frames[next_frame_pos].en == CAM_FRAME_AVAILABLE) {
            *frame_pos = next_frame_pos;
            return true;
        }

        return false;
    }
    return true;
}

static int draw_call(JPEGDRAW* pDraw) {
    if (!convert_pause || xSemaphoreTake(convert_pause, 0) == pdTRUE) {
        // Set fail reason
        bool* abort = pDraw->pUser;
        *abort = true;
        return 0;
    }

    uint32_t width = preview_img_width;
    uint16_t* off_buf = (uint16_t*)preview_frame_buf + (pDraw->x + pDraw->y * width);
    uint16_t* off_pix = pDraw->pPixels;
    size_t row_len = pDraw->iWidth * 2;

    // ESP_LOGI(TAG, "Draw %p %d %d %d %d", off_buf, pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);

    for (int i = 0; i < pDraw->iHeight; ++i) {
        // ESP_LOGI(TAG, "off_buf: %p, off_pix: %p, row_len: %zu", off_buf, off_pix, row_len);
        memcpy(off_buf, off_pix, row_len);
        off_buf += width;
        off_pix += pDraw->iWidth;
    }

    // static int a = 0;
    // for (int i = 0; i < pDraw->iWidth * pDraw->iHeight; ++i) {
    //     *off_buf &= a == 0 ? 0b1111100000000000 : a == 1 ? 0b0000011111100000 : 0b0000000000011111;
    //     off_buf--;
    // }
    // if (pDraw->y == 0)
    //     a = (a + 1) % 3;
    return 1;
}

esp_err_t camera_convert_and_send_preview(uint32_t width, uint32_t height, uint8_t* buf, size_t len, int jpg_scale) {
    // Check if image change
    if (jpg_scale) {
        width /= jpg_scale;
        height /= jpg_scale;
    }
    if (width != preview_img_width || height != preview_img_height) {
        preview_img_width = width;
        preview_img_height = height;

        preview_image_size_change(preview_img_width, preview_img_height);
    }

    bool abort = false;
    JPEGIMAGE jpeg;
    JPEG_openRAM(&jpeg, buf, (int)len, draw_call);
    JPEG_setPixelType(&jpeg, RGB565_LITTLE_ENDIAN);
    jpeg.pUser = &abort;
    // JPEG_decode return 1 if success
    esp_err_t success = !JPEG_decode(&jpeg, 0, 0, jpg_scale);
    JPEG_close(&jpeg);

    // Check if convert is abort
    if (abort) return CAMERA_CONVERT_ABORT;
    return success;
}

static void camera_convert_task(void* p) {
    int64_t last_time = esp_timer_get_time();
    int64_t fps_time = esp_timer_get_time();
    int64_t file_size = 0;
    int frame_count = 0;
    int frame_pos;

    while (cam_obj) {
        if (xQueueReceive(camera_frame_queue, &frame_pos, portMAX_DELAY) == pdFAIL)
            break;
        const camera_fb_t* fb = &cam_obj->frames[frame_pos].fb;

        // if (xSemaphoreTake(preview_update_semaphore, portMAX_DELAY) != pdPASS) {
        //     ESP_LOGE(TAG, "Failed to wait preview done");
        //     break;
        // }

        // int off = cam_verify_jpeg_eoi(fb->buf, fb->len);
        // if (off == -1) {
        //     xQueueSend(camera_frame_free_queue, &frame_pos, portMAX_DELAY);
        //     ESP_LOGW(TAG, "JPEG no eoi error");
        //     continue;
        // }
        // fb->len = off + 2;

        // ESP_LOGI(TAG, "convert_frame %d %d ", frame_pos, frame_count);

        file_size += fb->len;
        const esp_err_t res = camera_convert_and_send_preview(fb->width, fb->height, fb->buf, fb->len, jpg_scale);
        if (res == ESP_OK)
            xSemaphoreGive(preview_ready_semaphore);
        else if (res == CAMERA_CONVERT_ABORT)
            ESP_LOGW(TAG, "JPEG convert abort");
        else
            ESP_LOGE(TAG, "JPEG decode error");
        xQueueSend(camera_frame_free_queue, &frame_pos, portMAX_DELAY);

        frame_count++;
        const int64_t now = esp_timer_get_time();
        // Calculate delay
        const int64_t delay_time = (1000 / CAM_MAX_FRAMERATE) - (now - last_time) / 1000;
        if (delay_time > 0)
            vTaskDelay(pdMS_TO_TICKS(delay_time));
        last_time = now;

        // Print info
        if (now - fps_time > 2 * 1000 * 1000) {
            float t = 1000.f * 1000 / (float)(now - fps_time);
            char c[300];
            vTaskGetRunTimeStats(c);
            ESP_LOGI(TAG, "%.2ffps %.2fkb\n%s", frame_count * t, (float)file_size / frame_count / 1024.f, c);

            frame_count = 0;
            file_size = 0;
            fps_time = now;
        }
    }
    vTaskDelete(NULL);
}

void camera_set_resolution(const int quality, const framesize_t frame_size) {
    const uint8_t drop_frame = 10;
    xQueueOverwrite(camera_resolution_change, &drop_frame);

    ESP_LOGI(TAG, "Camera res %ux%u", resolution[frame_size].width, resolution[frame_size].height);

    sensor_t* s = esp_camera_sensor_get();
    s->set_quality(s, quality);
    s->set_framesize(s, frame_size);
    if (frame_size > FRAMESIZE_HVGA)
        s->set_reg(s, 0xd3, 0xff, 2); //clock

    xQueueOverwrite(camera_resolution_change, &drop_frame);
}

void camera_reset_resolution() {
    camera_set_resolution(camera_config.jpeg_quality, camera_config.frame_size);
}

void camera_capture(const capture_callback callback) {
    if (!cam_obj)
        return;
    atomic_ptr_set(&shared_capture_callback, callback);
}

void change_resolution() {
    // esp_camera_load_from_nvs("camera_settings");
    if (jpg_scale == CAM_PREVIEW_SCALE) {
        camera_set_resolution(CAM_CAPTURE_QUALITY, CAM_CAPTURE_SIZE);
        jpg_scale = CAM_CAPTURE_SCALE;
    } else if (jpg_scale == CAM_CAPTURE_SCALE) {
        camera_set_resolution(CAM_PREVIEW_QUALITY, CAM_PREVIEW_SIZE);
        jpg_scale = CAM_PREVIEW_SCALE;
    }
}

bool camera_is_on() {
    return cam_obj;
}

esp_err_t camera_start() {
    ESP_LOGI(TAG, "Camera starting...");
    
    convert_pause = xSemaphoreCreateBinary();
    preview_ready_semaphore = xSemaphoreCreateBinary();
    // preview_update_semaphore = xSemaphoreCreateBinary();
    camera_resolution_change = xQueueCreate(1, sizeof(uint8_t));
    camera_frame_queue = xQueueCreate(camera_config.fb_count, sizeof(int));
    camera_frame_free_queue = xQueueCreate(camera_config.fb_count, sizeof(int));

    esp_err_t res = esp_camera_init(&camera_config);
    if (res != ESP_OK)
        return res;

    sensor_t* s = esp_camera_sensor_get();
    s->set_brightness(s, 0); // -2 to 2
    s->set_contrast(s, 0); // -2 to 2
    s->set_saturation(s, 0); // -2 to 2
    // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_special_effect(s, 0);
    s->set_whitebal(s, 1); // 0 = disable , 1 = enable

    s->set_awb_gain(s, 1); // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0); // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)

    s->set_exposure_ctrl(s, 1); // 0 = disable , 1 = enable

    s->set_aec2(s, 0); // 0 = disable , 1 = enable
    s->set_ae_level(s, 0); // -2 to 2
    s->set_aec_value(s, 600); // 0 to 1200

    s->set_gain_ctrl(s, 0); // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0); // 0 to 30
    // s->set_gainceiling(s, (gainceiling_t)GAINCEILING_2X); // 0 to 6
    s->set_gainceiling(s, (gainceiling_t)GAINCEILING_8X); // 0 to 6

    s->set_bpc(s, 0); // 0 = disable , 1 = enable
    s->set_wpc(s, 1); // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1); // 0 = disable , 1 = enable
    s->set_lenc(s, 1); // 0 = disable , 1 = enable
    s->set_hmirror(s, 0); // 0 = disable , 1 = enable
    s->set_vflip(s, 0); // 0 = disable , 1 = enable
    s->set_dcw(s, 1); // 0 = disable , 1 = enable
    s->set_colorbar(s, 0); // 0 = disable , 1 = enable
    esp_camera_save_to_nvs("camera_settings");

    xTaskCreatePinnedToCore(camera_convert_task, "cam_conv", 40960, NULL, configMAX_PRIORITIES - 3, NULL, 1);

    ESP_LOGI(TAG, "Camera started");
    return ESP_OK;
}

esp_err_t camera_stop() {
    ESP_LOGI(TAG, "Camera stopping...");
    
    const esp_err_t ret = esp_camera_deinit();
    if (ret != ESP_OK)
        return ret;

    vSemaphoreDelete(convert_pause);
    convert_pause = NULL;

    vSemaphoreDelete(preview_ready_semaphore);
    preview_ready_semaphore = NULL;

    vQueueDelete(camera_resolution_change);
    camera_resolution_change = NULL;

    vQueueDelete(camera_frame_queue);
    camera_frame_queue = NULL;

    vQueueDelete(camera_frame_free_queue);
    camera_frame_free_queue = NULL;

    ESP_LOGI(TAG, "Camera stopped");
    return ret;
}
