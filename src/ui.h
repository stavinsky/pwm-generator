#pragma once

#include "lcd.h"
#include "lvgl.h"
#include "stdbool.h"

extern lv_obj_t * test_label;
extern bool button_pressed;
extern int freq_val, duty_val;
extern lv_obj_t *ta_freq;

void ui_init(void);
bool encoder_read(lv_indev_drv_t * drv, lv_indev_data_t*data);
int enc_get_new_moves(void );
