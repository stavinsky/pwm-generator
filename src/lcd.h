#pragma once

#include "stdbool.h"
#include "stdint.h"
#include "lvgl.h"
#include <libopencm3/stm32/gpio.h>
#define RST 1<<1
#define LED 1<<0
#define RD 1<<10
#define WR 1<<11
#define CS 1<<8
#define RS 1<<9
#define DISP_PORT GPIOA

#define disp_size_x 239
#define disp_size_y 319

#define set_high(reg, bit) reg |= bit
#define set_low(reg, bit) reg &= ~(bit)

#define pulse_low(reg, bit) set_low(reg, bit); set_high(reg, bit);

void lcd_init(void);

void lcd_set_xy(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void lcd_fill_rect(long pix);
void lcd_set_pixel_color(unsigned int color);

void lcd_set_color(uint8_t r, uint8_t g, uint8_t b);

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
