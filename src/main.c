#include "main.h"
#include "lvgl.h"
#include "stdbool.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/exti.h>
#include "utils.h"
#include "ui.h"
#include "pwm.h"
#include "stdlib.h"
void sys_tick_handler(void)
{
    lv_tick_inc(1);
	system_millis++;
}
bool button_pressed = false;
void exti9_5_isr(void)
{
    exti_reset_request(EXTI7);

    if (button_pressed == false) {
        button_pressed= true;
        exti_set_trigger(EXTI7, EXTI_TRIGGER_RISING);
    } else {
        button_pressed= false;
        exti_set_trigger(EXTI7, EXTI_TRIGGER_FALLING);
    }
}

void encoder_button_init(void){
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);

    /* Enable EXTI0 interrupt. */
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);

    /* Set GPIO0 (in GPIO port A) to 'input open-drain'. */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO7);

    /* Configure the EXTI subsystem. */
    exti_select_source(EXTI7, GPIOB);
    exti_set_trigger(EXTI7, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI7);

}
int main(void){
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    systick_set_frequency(1000, 36000000);
    systick_counter_enable();
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0);
    systick_interrupt_enable();
    encoder_button_init();
	pwm_init();
	ui_init();
    /*int diff = 0;*/
    while (1) {
		lv_task_handler();
		msleep(15);
        set_period(freq_val, duty_val);
        /*diff= enc_get_new_moves();*/
        /*if (abs(diff) >  0){*/
        /*    lv_label_set_text_fmt(test_label, "   %d ", diff);*/
        /*}*/
        /*[>__asm__("wfe");<]*/
    }

}


