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

#define disp_size_x 240 
#define disp_size_y 320

#define LCD_DATA  (*(volatile uint8_t *)0x60010000)
#define LCD_REG  (*(volatile uint8_t *)0x60000000 )

#define set_high(reg, bit) GPIO_BSRR(reg) = bit
#define set_low(reg, bit) GPIO_BSRR(reg) = (bit) << 16
#define write_com(VL) LCD_REG=0x00; LCD_REG=VL;
#define write_data(VH, VL) LCD_DATA=VH; LCD_DATA=VL; 
#define write_com_data(com, data) write_com(com); write_data(((data)>>8), (( data)&0x00FF));
#define write_data_16(data) write_data(((data)>>8), (data&0x00FF)); 
void lcd_init(void);

void lcd_set_xy(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

void lcd_fill_rect(long pix);
void lcd_set_pixel_color(unsigned int color);

void lcd_set_color(uint8_t r, uint8_t g, uint8_t b);

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
void test_fill_display(uint16_t);
void test_fill_display2(uint16_t *color_p, uint16_t x1,uint16_t x2,uint16_t y1,uint16_t y2);
void tm_start(int);
void dma_timer_init(void);
void my_flush_cb2(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p);
void dma_init(void);