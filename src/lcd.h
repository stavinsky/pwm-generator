#pragma once

#include "stdbool.h"
#include "stdint.h"
#include "lvgl.h"
#include <libopencm3/stm32/gpio.h>

#define RST GPIO7
#define RST_PORT GPIOB

#define LED GPIO12
#define LED_PORT GPIOC

#define RD GPIO6
#define RD_PORT GPIOB

#define WR GPIO5
#define WR_PORT GPIOB

#define CS GPIO3
#define CS_PORT GPIOB

#define RS GPIO4
#define RS_PORT GPIOB

#define DISP_PORT GPIOD

#define disp_size_x 239
#define disp_size_y 319


#define set_high(reg, bit) GPIO_BSRR(reg) = bit
#define set_low(reg, bit) GPIO_BSRR(reg) = (bit) << 16
#define pulse_low(port, bit) set_low(port, bit); set_high(port, bit);
#define write_bus_8(bus)     GPIO_BSRR(DISP_PORT)  = (bus) | 0x00FF0000; pulse_low(WR_PORT, WR);
#define write_bus(VH, VL) write_bus_8(VH); write_bus_8(VL);
#define write_bus_16(data) write_bus_8((data)>>8); write_bus_8(data&0x00FF);
#define write_com(VL) set_low(RS_PORT, RS); write_bus(0x00,VL); set_high(RS_PORT, RS);
#define write_data(VH, VL) write_bus(VH, VL);
#define write_com_data(com, data) write_com(com); write_data((data)>>8, data&0x00FF);

void lcd_init(void);

void lcd_set_xy(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void lcd_fill_rect(long pix);
void lcd_set_pixel_color(unsigned int color);

void lcd_set_color(uint8_t r, uint8_t g, uint8_t b);

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
