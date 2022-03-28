#include "ui.h"
#include "pwm.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <stdio.h>

lv_obj_t * ta_freq;
lv_obj_t * ta_duty;

lv_obj_t * test_label;

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
    lv_label_set_text_fmt(test_label, "   %d ", counter);
    return diff;
}

static void freq_event_cb(lv_obj_t * obj, lv_event_t event){
    switch(event) {
        case LV_EVENT_KEY:
            {
                const uint32_t * key = lv_event_get_data();
                if ((*key == 19) & (freq_val >0)){
                    freq_val--;
                }
                if ((*key == 20) & (freq_val <500))
                {
                    freq_val++;
                }
                char buff[20] = "";
                sprintf(buff, "freq %d", freq_val);
                set_period(freq_val, duty_val);
                lv_textarea_set_text(obj, buff);
                break;
            }

    }
}

static void duty_event_cb(lv_obj_t * obj, lv_event_t event){
    switch(event) {
        case LV_EVENT_KEY:
            {
                const uint32_t * key = lv_event_get_data();
                if ((*key == 19) & (duty_val >0) ){
                    duty_val--;
                }
                if ((*key == 20) & (duty_val <99))
                {
                    duty_val++;
                }
                char buff[20] = "";
                sprintf(buff, "duty %d", duty_val);
                lv_textarea_set_text(obj, buff);
                set_period(freq_val, duty_val);
                break;
            }

    }
}

bool encoder_read(lv_indev_drv_t * drv, lv_indev_data_t*data){
  data->enc_diff = enc_get_new_moves();
  if(button_pressed == true ) {
    data->state = LV_INDEV_STATE_PR;
  }
  else data->state = LV_INDEV_STATE_REL;
  /*data->state = LV_INDEV_STATE_REL;*/

  return false; /*No buffering now so no more data read*/
}
void encoder_init(void )
{
	timer_disable_counter(TIM4);
    rcc_periph_clock_enable(RCC_GPIOD);
	// gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_TIM4_REMAP );
	rcc_periph_clock_enable(RCC_TIM4);
	timer_set_period(TIM4, 4096);
	timer_slave_set_mode(TIM4, 0x3);
	timer_ic_set_input(TIM4, TIM_IC1, TIM_IC_IN_TI1);
	timer_ic_set_input(TIM4, TIM_IC2, TIM_IC_IN_TI2);
    timer_set_counter(TIM4, 0);
	timer_enable_counter(TIM4);

    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = encoder_read;
    encoder_indev = lv_indev_drv_register(&indev_drv);

}

void ui_init(void){
	lv_init();
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, CS|WR|RS|RST|RD);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED);
	gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, 0xFF);


	lcd_init();

	lcd_set_color(0,255,0);
	lcd_set_xy(0,0,disp_size_x, disp_size_y);
	lcd_fill_rect(disp_size_x*disp_size_y+2);


	static lv_disp_buf_t draw_buf_dsc_1;
	static lv_color_t draw_buf_1[LV_HOR_RES_MAX * 20];
	lv_disp_buf_init(&draw_buf_dsc_1, draw_buf_1, NULL, LV_HOR_RES_MAX * 20);


	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);

	disp_drv.hor_res = 240;
	disp_drv.ver_res = 320;
	disp_drv.antialiasing=1;

	disp_drv.flush_cb = my_flush_cb;

	disp_drv.buffer = &draw_buf_dsc_1;
	lv_disp_drv_register(&disp_drv);

	lv_obj_t * scr1 = lv_obj_create(NULL, NULL);
	lv_scr_load(scr1);

    encoder_init();
    g = lv_group_create();
    lv_indev_set_group(encoder_indev, g);

	ta_freq = lv_textarea_create(lv_scr_act(), NULL);
	lv_obj_set_size(ta_freq, 200, 80);
    lv_obj_set_event_cb(ta_freq, freq_event_cb);
    lv_textarea_set_text(ta_freq, "freq 0");
    lv_group_add_obj(g, ta_freq);

	ta_duty = lv_textarea_create(lv_scr_act(), NULL);
	lv_obj_align(ta_duty, ta_freq, LV_ALIGN_OUT_BOTTOM_LEFT, 0,10);
	lv_obj_set_size(ta_duty, 200, 50);
    lv_textarea_set_text(ta_duty, "duty 0");
    lv_obj_set_event_cb(ta_duty, duty_event_cb);
    lv_group_add_obj(g, ta_duty);

    test_label = lv_label_create(lv_scr_act(), NULL);
    lv_obj_align(test_label, NULL, LV_ALIGN_IN_BOTTOM_LEFT, 0, -5);
    lv_label_set_text(test_label, "encoder");

}



