#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#include <lvgl__lvgl/lvgl.h>

extern int32_t lcd_width, lcd_height;

void lv_port_disp_init(void);
void lcd_backlight_on(void);
lv_color_t lv_color_hex_16(uint32_t c);

#endif /*LV_PORT_DISP_H*/
