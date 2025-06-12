#include "lv_port_disp.h"

#include <stdbool.h>

#include <esp_lcd_touch_cst816s.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>
#include <esp_log.h>
#include <driver/i2c_master.h>

/*********************
 * LCD hardware specs
 *********************/
#define LCD_HOR_RES     240
#define LCD_VER_RES     280
#define LCD_HOR_OFF     0
#define LCD_VER_OFF     20

/*********************
 *    LCD settings
 *********************/
#define LCD_SPI_HOST    SPI2_HOST
#define LCD_RST_PIN     GPIO_NUM_NC
#define LCD_DC_PIN      3
#define LCD_CS_PIN      4
#define LCD_MOSI_PIN    9
#define LCD_CLK_PIN     7
#define LCD_BK_PIN      2
#define LCD_ROTATION    LV_DISPLAY_ROTATION_90
#define LCD_COLOR_FMT   LV_COLOR_FORMAT_RGB565

/*********************
 * LCD touch settings
 *********************/
#define TOUCH_I2C_HOST  0
#define TOUCH_SDA_PIN   5
#define TOUCH_SCL_PIN   6
#define TOUCH_RST_PIN   44
#define TOUCH_INT_PIN   43
#define TOUCH_OFF_X     2
#define TOUCH_OFF_X1    278
#define TOUCH_OFF_Y     10
#define TOUCH_OFF_Y1    235

#define LCD_BYTE_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LCD_COLOR_FMT))
#define LVGL_BUFFER_SIZE ((LCD_HOR_RES * LCD_VER_RES * LCD_BYTE_PER_PIXEL) / 6)

static const char* TAG = "LCD";
static uint8_t *lvgl_buf1 = NULL, *lvgl_buf2 = NULL;
static esp_lcd_panel_handle_t lcd_disp_handle = NULL;
static esp_lcd_touch_handle_t lcd_touch_handle = NULL;
static lv_display_t* lcd_disp;

static SemaphoreHandle_t touch_mux;

int32_t lcd_width, lcd_height;

esp_err_t lcd_backlight_init() {
    gpio_config_t bk_cfg = {
        .pin_bit_mask = (1ULL << LCD_BK_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&bk_cfg);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Backlight GPIO configured, setting high");
    }
    return ret;
}

static void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* data) {
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // ESP_LOGI("LVGL", "Flushing area %d %d %d %d", offsetx1, offsety1, offsetx2, offsety2);

    int pixel_count = (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1);
    lv_draw_sw_rgb565_swap(data, pixel_count);

    esp_err_t ret = esp_lcd_panel_draw_bitmap(lcd_disp_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw bitmap: %d", ret);
    }

    lv_display_flush_ready(disp);
}

/* Initialize the SPI bus and add the LCD device */
void lcd_panel_init(int32_t offset_x, int32_t offset_y,
                    bool swap, bool mirror_x,bool mirror_y) {
    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = LCD_MOSI_PIN,
        .miso_io_num = -1,
        .sclk_io_num = LCD_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    esp_err_t ret = spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed: %d", ret);
        return;
    }

    // Create LCD panel IO handle
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = LCD_CS_PIN,
        .dc_gpio_num = LCD_DC_PIN,
        .spi_mode = 0,
        .pclk_hz = 64 * 1000 * 1000,
        .trans_queue_depth = 4,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ret = esp_lcd_new_panel_io_spi(LCD_SPI_HOST, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create LCD IO handle: %d", ret);
        return;
    }

    // Initialize ST7789 panel
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST_PIN,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BYTE_PER_PIXEL * 8,
    };
    ret = esp_lcd_new_panel_st7789(io_handle, &panel_config, &lcd_disp_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ST7789 panel: %d", ret);
        return;
    }
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_disp_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_disp_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_disp_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_disp_handle, true));

    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_disp_handle, swap));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_disp_handle, mirror_x,mirror_y));

    // esp_lcd_panel_io_tx_param(io_handle, 0xF2, (uint8_t[]){0}, 1); // 3Gamma function disable
    // esp_lcd_panel_io_tx_param(io_handle, 0x26, (uint8_t[]){1}, 1); // Gamma curve 1 selected
    // esp_lcd_panel_io_tx_param(io_handle, 0xE0, (uint8_t[]){        // Set positive gamma
    //                                                        0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00},
    //                           15);
    // esp_lcd_panel_io_tx_param(io_handle, 0xE1, (uint8_t[]){// Set negative gamma
    //                                                        0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F},
    //                           15);

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_disp_handle, true));

    /*
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 12,
        .task_stack = 8192,
        .task_affinity = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 5,
    };
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    // Create LVGL display
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = LVGL_BUFFER_SIZE,
        .double_buffer = true,
        .hres = LCD_HOR_RES,
        .vres = LCD_VER_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .direct_mode = false,
            .full_refresh = false,
        },
    };
    lcd_disp = lvgl_port_add_disp(&disp_cfg);
    */

    lcd_disp = lv_display_create(lcd_width, lcd_height);
    lv_display_set_color_format(lcd_disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(lcd_disp, lvgl_flush_cb);
    lv_display_set_offset(lcd_disp, offset_x, offset_y);
    lv_display_set_user_data(lcd_disp, lcd_disp_handle);

    // Set up buffers
    lvgl_buf1 = heap_caps_aligned_alloc(4, LVGL_BUFFER_SIZE, MALLOC_CAP_DMA);
    lvgl_buf2 = heap_caps_aligned_alloc(4, LVGL_BUFFER_SIZE, MALLOC_CAP_DMA);
    lv_display_set_buffers(lcd_disp, lvgl_buf1, lvgl_buf2, LVGL_BUFFER_SIZE, LV_DISPLAY_RENDER_MODE_PARTIAL);

    ESP_LOGI(TAG, "Lcd_panel init complete");
}

void touch_callback(esp_lcd_touch_handle_t tp) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(touch_mux, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void touchpad_read(lv_indev_t* indev, lv_indev_data_t* data) {
    if (xSemaphoreTake(touch_mux, 0) == pdTRUE) {
        esp_lcd_touch_read_data(lcd_touch_handle);
    }
    uint16_t touch_x, touch_y;
    uint16_t touch_strength;
    uint8_t touch_cnt = 0;

    bool touchpad_pressed = esp_lcd_touch_get_coordinates(lcd_touch_handle, &touch_x, &touch_y, &touch_strength,
                                                          &touch_cnt, 1);

    // ESP_LOGI(TAG, "Touch: %d, %d, %d, %d", touchX, touchY, strength, touched);
    // uint16_t touchX, touchY, strength;
    // uint8_t point_num;
    // esp_lcd_touch_get_coordinates(lcd_touch_handle, &touchX, &touchY, &strength, &point_num,
    //                               CONFIG_ESP_LCD_TOUCH_MAX_POINTS);
    // esp_lcd_touch_get_button_state(lcd_touch_handle, point_num, (uint8_t*)&touched);

    if (!touchpad_pressed) {
        data->state = LV_INDEV_STATE_REL;
    } else {
        const int32_t touch_x_map = lcd_width * touch_x / (TOUCH_OFF_X1 - TOUCH_OFF_X) - TOUCH_OFF_X;
        const int32_t touch_y_map = lcd_height * touch_y / (TOUCH_OFF_Y1 - TOUCH_OFF_Y) - TOUCH_OFF_Y;
        if (touch_x_map < 0 || touch_x_map >= lcd_width ||
            touch_y_map < 0 || touch_y_map >= lcd_height) {
            data->state = LV_INDEV_STATE_REL;
            return;
        }


        ESP_LOGI(TAG, "Touch: %d, %ld, %ld", touch_cnt, touch_x_map, touch_y_map);
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touch_x_map;
        data->point.y = touch_y_map;
    }
}

void lcd_touch_init(bool swap, bool mirror_x,bool mirror_y) {
    i2c_master_bus_handle_t i2c_bus = NULL;
    const i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = TOUCH_I2C_HOST,
        .sda_io_num = TOUCH_SDA_PIN,
        .scl_io_num = TOUCH_SCL_PIN,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_CST816S_ADDRESS,
        .on_color_trans_done = 0,
        .user_ctx = NULL,
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 0,
        .flags =
        {
            .dc_low_on_data = 0,
            .disable_control_phase = 1,
        },
        .scl_speed_hz = 400 * 1000
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    touch_mux = xSemaphoreCreateBinary();

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_HOR_RES,
        .y_max = LCD_VER_RES,
        .rst_gpio_num = TOUCH_RST_PIN,
        .int_gpio_num = TOUCH_INT_PIN,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = swap,
            .mirror_x = mirror_x,
            .mirror_y = mirror_y,
        },
        .interrupt_callback = touch_callback,
    };

    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst816s(io_handle, &tp_cfg, &lcd_touch_handle));

    lv_indev_t* indev_drv = lv_indev_create();
    lv_indev_set_type(indev_drv, LV_INDEV_TYPE_POINTER);
    lv_indev_set_display(indev_drv, lcd_disp);
    lv_indev_set_read_cb(indev_drv, touchpad_read);

    ESP_LOGI(TAG, "Touch init complete");
}


void lv_port_disp_init() {
    int32_t offset_x, offset_y;
    if (LCD_ROTATION == LV_DISPLAY_ROTATION_90 || LCD_ROTATION == LV_DISPLAY_ROTATION_270) {
        lcd_width = LCD_VER_RES;
        lcd_height = LCD_HOR_RES;
        offset_x = LCD_VER_OFF;
        offset_y = LCD_HOR_OFF;
    } else {
        lcd_width = LCD_HOR_RES;
        lcd_height = LCD_VER_RES;
        offset_x = LCD_HOR_OFF;
        offset_y = LCD_VER_OFF;
    }
    const bool swap = LCD_ROTATION == LV_DISPLAY_ROTATION_90 || LCD_ROTATION == LV_DISPLAY_ROTATION_270;
    const bool mirror_x = LCD_ROTATION == LV_DISPLAY_ROTATION_180 || LCD_ROTATION == LV_DISPLAY_ROTATION_90;
    const bool mirror_y = LCD_ROTATION == LV_DISPLAY_ROTATION_180 || LCD_ROTATION == LV_DISPLAY_ROTATION_270;

    lv_init();
    lcd_panel_init(offset_x, offset_y, swap, mirror_x, mirror_y);
    lcd_backlight_init();

    lcd_touch_init(swap, mirror_x, mirror_y);
}

void lcd_backlight_on(void) {
    gpio_set_level(LCD_BK_PIN, 1);
}
