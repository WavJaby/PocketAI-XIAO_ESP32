//
// Created by user on 2025/4/11.
//

#ifndef CAMERA_H
#define CAMERA_H

#include <esp_camera.h>
#include <freertos/FreeRTOS.h>

#define CAMERA_CONVERT_ABORT 1

extern uint8_t* preview_frame_buf;
extern SemaphoreHandle_t preview_ready_semaphore;
extern SemaphoreHandle_t preview_update_semaphore;
extern uint32_t preview_img_width, preview_img_height;
extern void preview_image_size_change(uint32_t width, uint32_t height);

typedef void (*capture_callback)(camera_fb_t* fb);

esp_err_t camera_start();
esp_err_t camera_stop();
bool camera_is_on();
void camera_set_resolution(int quality, framesize_t frame_size);
void camera_reset_resolution();
void camera_capture(capture_callback callback);
void change_resolution();
esp_err_t camera_convert_and_send_preview(uint32_t width, uint32_t height, uint8_t* buf, size_t len, int jpg_scale);

#endif //CAMERA_H
