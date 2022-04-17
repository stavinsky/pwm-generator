#include "ui.h"
#include "pwm.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <stdio.h>

// lv_obj_t * ta_freq;
// lv_obj_t * ta_duty;

lv_obj_t * test_label;
lv_obj_t * duty_slider;
lv_obj_t * duty_slider_label;
lv_obj_t * freq_slider;
lv_obj_t * freq_slider_label;
lv_disp_drv_t disp_drv;


int freq_val = 0;
int duty_val = 0;
lv_group_t* g; //An Object Group
lv_indev_t* encoder_indev; //The input device

int enc_get_new_moves(void){
    int counter;
    static int old=0;
    int diff=0;
    bool dir_down = (TIM4_CR1 & 1<<4) > 0;
    counter = timer_get_counter(TIM4)/4;
    diff = counter - old;
    if (dir_down & (diff >0)){
        diff = 1024 - diff - 1;
    }
    if ((!dir_down) & (diff < 0)){
        diff = 1024 + diff +1;
    }

    old = counter;
    // lv_label_set_text_fmt(test_label, "   %d ", counter);
    return diff;
}
static void freq_slider_event_cb(lv_event_t *event) {
    lv_obj_t * slider = lv_event_get_target(event);
    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d%%", (int)lv_slider_get_value(slider));
    lv_label_set_text(freq_slider_label, buf);

}
static void duty_slider_event_cb(lv_event_t *event) {
    lv_obj_t * slider = lv_event_get_target(event);
    char buf[8];
    lv_snprintf(buf, sizeof(buf), "%d%%", (int)lv_slider_get_value(slider));
    lv_label_set_text(duty_slider_label, buf);

}

void encoder_read(lv_indev_drv_t * drv, lv_indev_data_t*data){
  data->enc_diff = enc_get_new_moves();
  if(button_pressed == true ) {
    data->state = LV_INDEV_STATE_PRESSED;
  }
  else data->state = LV_INDEV_STATE_RELEASED;
}
void encoder_init(void )
{
	timer_disable_counter(TIM4);
    rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_TIM4);
	timer_set_period(TIM4, 4096);
	timer_slave_set_mode(TIM4, 0x3);
	timer_ic_set_input(TIM4, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM4, TIM_IC2, TIM_IC_IN_TI2);
    timer_set_counter(TIM4, 0);
	timer_enable_counter(TIM4);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = encoder_read;
    encoder_indev = lv_indev_drv_register(&indev_drv);

}

void ui_init(void){
	lv_init();
	// rcc_periph_clock_enable(RCC_GPIOD);
	// rcc_periph_clock_enable(RCC_GPIOB);
	// rcc_periph_clock_enable(RCC_GPIOC);

	// gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, CS|WR|RS|RST|RD);
	// gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED);
	// gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, 0xFF);


	lcd_init();

	lcd_set_color(0,255,0);
	lcd_set_xy(0,0,disp_size_x, disp_size_y);
	lcd_fill_rect(disp_size_x*disp_size_y+2);


	static lv_disp_draw_buf_t disp_buf;
	static lv_color_t buf_1[disp_size_x * 50];
	// static lv_color_t buf_2[disp_size_x * 20];
	lv_disp_draw_buf_init(&disp_buf, buf_1, NULL, disp_size_x * 50);


	// static lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);

	disp_drv.antialiasing=1;

	disp_drv.draw_buf = &disp_buf;
	disp_drv.flush_cb = my_flush_cb;
	disp_drv.hor_res = disp_size_x;
	disp_drv.ver_res = disp_size_y;

    lv_disp_t * disp;
	disp = lv_disp_drv_register(&disp_drv);

	lv_obj_t * scr1 = lv_obj_create(NULL);
	lv_scr_load(scr1);

    encoder_init();
    g = lv_group_create();
    lv_indev_set_group(encoder_indev, g);
    lv_group_set_default(g);

    freq_slider = lv_slider_create(lv_scr_act());
    lv_obj_add_event_cb(freq_slider, freq_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_slider_set_range(freq_slider, 0, 500);
    freq_slider_label = lv_label_create(lv_scr_act());
    lv_label_set_text(freq_slider_label, "0%");
    lv_obj_align_to(freq_slider_label, freq_slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    duty_slider = lv_slider_create(lv_scr_act());
    lv_obj_add_event_cb(duty_slider, duty_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_align_to(duty_slider, freq_slider, LV_ALIGN_BOTTOM_LEFT, 0, 50);
    lv_slider_set_range(duty_slider, 0, 100);
    duty_slider_label = lv_label_create(lv_scr_act());
    lv_label_set_text(duty_slider_label, "0%");
    lv_obj_align_to(duty_slider_label, duty_slider, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    test_label = lv_label_create(lv_scr_act());
    lv_obj_align(test_label, LV_ALIGN_OUT_TOP_LEFT, 0, 210);
    lv_label_set_text(test_label, "encoder");

}



