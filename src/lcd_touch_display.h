#pragma once
#include "stdint.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "pin_config.h"
#include "ui/ui.h"
#include "XL9535_driver.h"

// #define USING_2_1_INC_CST820     1           //  Full circle 2.1 inches using CST820 touch screen
// #define USING_2_8_INC_GT911      1           //  Full circle 2.8 inches using GT911 touch screen
// #define USING_2_1_INC_FT3267     1           //  Half circle 2.1 inches use FT3267 touch screen

#if !defined(USING_2_1_INC_CST820) && !defined(USING_2_8_INC_GT911) && !defined(USING_2_1_INC_FT3267)
#error "Please define the size of the screen and open the macro definition at the top of the sketch"
#endif

typedef struct {
    uint16_t x;
    uint16_t y;
} touch_point_t;

typedef struct
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

#define FT5x06_ADDR 0x38
#define CST820_ADDR 0x15
#define GT911_ADDR  0x5A

esp_lcd_panel_handle_t lcd_touch_setup();
const char *getTouchAddr();
void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
void lv_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data);
void waitInterruptReady();
void setBrightness(uint8_t value);