//
// Created by user on 2025/4/22.
//
#include "main_ui.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_camera.h>
#include <JPEGDEC.h>
#include <lvgl__lvgl/lvgl.h>
#include <freertos/FreeRTOS.h>

#include "camera.h"
#include "lv_port_disp.h"
#include "misc/lv_event_private.h"

static const char* TAG = "MainUI";

uint8_t* preview_frame_buf = NULL;
lv_obj_t* canvas = NULL;
bool update_preview_canvas_size = false;

void preview_image_size_change(uint32_t width, uint32_t height) {
    if (preview_frame_buf)
        heap_caps_free(preview_frame_buf);

    const size_t canvas_buf_size = LV_CANVAS_BUF_SIZE(width, height, 2 * 8, LV_DRAW_BUF_STRIDE_ALIGN);
    preview_frame_buf = heap_caps_malloc(canvas_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    ESP_LOGI(TAG, "prev %lux%lu %u %p", width, height, canvas_buf_size, preview_frame_buf);
    
    update_preview_canvas_size = true;
}

lv_anim_t shutter_animation;

void camera_capture_callback(camera_fb_t* frame) {
    ESP_LOGI(TAG, "Captured size: %dx%d %u", frame->width, frame->height, frame->len);

    // if (camera_convert_and_send_preview(frame->width, frame->height, frame->buf, frame->len, JPEG_SCALE_EIGHTH))
    //     xSemaphoreGive(preview_ready_semaphore);
}

void shutter_btn_event_cb(lv_event_t* lv_event) {
    camera_capture(camera_capture_callback);

    lv_anim_start(&shutter_animation);
    // change_resolution();
}

void update_preview_canvas() {
    if (!preview_ready_semaphore || xSemaphoreTake(preview_ready_semaphore, 0) == pdFAIL)
        return;


    if (update_preview_canvas_size) {
        update_preview_canvas_size = false;
        ESP_LOGI(TAG, "Update preview canvas size");

        lv_canvas_set_buffer(canvas, preview_frame_buf,
                             (int32_t)preview_img_width, (int32_t)preview_img_height, LV_COLOR_FORMAT_RGB565);
        lv_obj_set_pos(canvas,
                       -((int32_t)preview_img_width - lcd_width) / 2, -((int32_t)preview_img_height - lcd_height) / 2);
        return;
    }

    // Update preview canvas
    lv_obj_invalidate(canvas);
}

static void shutter_animation_cb(void* var, int32_t v) {
    lv_obj_set_style_bg_opa(var, v, 0);
    if (v == 0)
        lv_obj_add_flag(var, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_remove_flag(var, LV_OBJ_FLAG_HIDDEN);
}

void camera_switch_cb(lv_event_t* lv_event) {
    ESP_LOGI(TAG, "Camera switch event");
    lv_obj_t* camera_switch = lv_event->user_data;
    if (lv_obj_has_state(camera_switch, LV_STATE_CHECKED))
        camera_start();
    else
        camera_stop();
}

void main_ui_init() {
    ESP_LOGI(TAG, "Main UI init");
    // Background
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(lv_screen_active(), LV_DIR_NONE);

    // Camera preview
    canvas = lv_canvas_create(lv_screen_active());

    // Shutter overlay
    lv_obj_t* shutter_overlay = lv_obj_create(lv_screen_active());
    lv_obj_set_scrollbar_mode(shutter_overlay, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scroll_dir(shutter_overlay, LV_DIR_NONE);
    lv_obj_set_style_bg_color(shutter_overlay, lv_color_black(), 0);
    lv_obj_set_style_radius(shutter_overlay, 0, 0);
    lv_obj_set_style_border_width(shutter_overlay, 0, 0);
    lv_obj_set_style_size(shutter_overlay, lcd_width, lcd_height, 0);
    lv_obj_add_flag(shutter_overlay, LV_OBJ_FLAG_HIDDEN);
    
    // Shutter animation
    lv_anim_init(&shutter_animation);
    lv_anim_set_var(&shutter_animation, shutter_overlay);
    lv_anim_set_values(&shutter_animation, 55, 255);
    lv_anim_set_duration(&shutter_animation, 200);
    lv_anim_set_playback_delay(&shutter_animation, 200);
    lv_anim_set_playback_duration(&shutter_animation, 200);
    lv_anim_set_path_cb(&shutter_animation, lv_anim_path_linear);
    lv_anim_set_exec_cb(&shutter_animation, shutter_animation_cb);

    // lv_obj_t* camera_switch = lv_switch_create(lv_screen_active());
    // lv_obj_align(camera_switch, LV_ALIGN_TOP_LEFT, 0, 0);
    // lv_obj_add_state(camera_switch, LV_STATE_CHECKED);
    // lv_obj_add_event_cb(camera_switch, camera_switch_cb, LV_EVENT_VALUE_CHANGED, camera_switch);

    // Shutter button
    lv_obj_t* shutter_button = lv_button_create(lv_screen_active());
    lv_obj_set_size(shutter_button, 50, 50);
    lv_obj_align(shutter_button, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_set_style_radius(shutter_button, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_ext_click_area(shutter_button, 5);
    lv_obj_add_event_cb(shutter_button, shutter_btn_event_cb, LV_EVENT_CLICKED, NULL);
}

void main_ui_frame_update() {
    update_preview_canvas();
}
