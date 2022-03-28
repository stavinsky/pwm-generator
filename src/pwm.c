#include "pwm.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

void pwm_init(void)
{
    /* Enable TIM1 clock. */
    rcc_periph_clock_enable(RCC_TIM1);

    /* Enable GPIOA, GPIOB and Alternate Function clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
              GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
              GPIO_TIM1_CH1 | GPIO_TIM1_CH2 );

    rcc_periph_reset_pulse(RST_TIM1);

    timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM1, 0);
    timer_set_repetition_counter(TIM1, 0);
    timer_enable_preload(TIM1);
    timer_continuous_mode(TIM1);

    timer_set_period(TIM1, 720);

    timer_disable_oc_output(TIM1, TIM_OC1);
    timer_disable_oc_output(TIM1, TIM_OC2);

    timer_disable_oc_clear(TIM1, TIM_OC1);
    timer_enable_oc_preload(TIM1, TIM_OC1);
    timer_set_oc_slow_mode(TIM1, TIM_OC1);
    timer_set_oc_mode(TIM1, TIM_OC1, TIM_OCM_PWM1);
    timer_disable_oc_clear(TIM1, TIM_OC2);
    /*timer_enable_oc_preload(TIM1, TIM_OC2);*/
    timer_set_oc_slow_mode(TIM1, TIM_OC2);
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM1);


    timer_set_oc_polarity_high(TIM1, TIM_OC1);
    timer_set_oc_idle_state_set(TIM1, TIM_OC1);

    timer_set_oc_polarity_low(TIM1, TIM_OC2);
    timer_set_oc_idle_state_set(TIM1, TIM_OC2);

    timer_set_oc_value(TIM1, TIM_OC1, 360);
    timer_set_oc_value(TIM1, TIM_OC2, 720-360);

    timer_enable_oc_output(TIM1, TIM_OC1);
    timer_enable_oc_output(TIM1, TIM_OC2);

    /* ARR reload enable. */
    timer_enable_preload(TIM1);

    /*
     * Enable preload of complementary channel configurations and
     * update on COM event.
     */
    timer_enable_preload_complementry_enable_bits(TIM1);

    /* Enable outputs in the break subsystem. */
    timer_enable_break_main_output(TIM1);

    /* Counter enable. */
    timer_enable_counter(TIM1);

    /* Enable commutation interrupt. */
    timer_enable_irq(TIM1, TIM_DIER_COMIE);
}
void set_period(int freq, int duty){
    static int cur_freq = 0;
    static int cur_duty = 0;
    if ((cur_duty == duty) & (cur_freq == freq)){
        return;
    }
    freq = freq * 1000 * 2;
    uint16_t period = 72000000 / freq;
    uint16_t cmp = (period / 100.0) * duty/2.;
    if((period>0) & (duty>0)){
        timer_set_period(TIM1, period);
        timer_set_oc_value(TIM1, TIM_OC1, cmp);
        timer_set_oc_value(TIM1, TIM_OC2, period - cmp);
    }

}
