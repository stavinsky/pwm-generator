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
#include <libopencm3/stm32/dma.h>
void encoder_button_init(void);
void sys_tick_handler(void)
{
    lv_tick_inc(1);
	system_millis++;
}
bool button_pressed = false;
void exti15_10_isr(void)
{
    exti_reset_request(EXTI14);

    if (button_pressed == false) {
        button_pressed= true;
        exti_set_trigger(EXTI14, EXTI_TRIGGER_RISING);
    } else {
        button_pressed= false;
        exti_set_trigger(EXTI14, EXTI_TRIGGER_FALLING);
    }
}

void encoder_button_init(void){
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_AFIO);

    //  Enable EXTI0 interrupt. 
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);

    // Set GPIO0 (in GPIO port A) to 'input open-drain'. 
    gpio_set_mode(GPIOD, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);

    // Configure the EXTI subsystem. 
    exti_select_source(EXTI14, GPIOD);
    exti_set_trigger(EXTI14, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI14);
    
}

int main(void){

    rcc_periph_clock_enable(RCC_AFIO);
	gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_SPI3_REMAP|AFIO_MAPR_TIM4_REMAP| AFIO_MAPR_TIM3_REMAP_PARTIAL_REMAP );
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    systick_set_frequency(1000, 36000000);
    systick_counter_enable();
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
    nvic_set_priority(NVIC_SYSTICK_IRQ, 1);
    systick_interrupt_enable();
    encoder_button_init();
	// pwm_init();
	ui_init();
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, WR|CS|RS|RST|RD);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED);
	gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, 0xFF);

    dma_timer_init();
    dma_init();
    while (1) {
        lv_task_handler();
        msleep(20);
    }

}


