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
void fsmc_init(void);
void sys_tick_handler(void)
{
    lv_tick_inc(1);
	system_millis++;
}
bool button_pressed = false;
void exti15_10_isr(void)
{
    exti_reset_request(EXTI10);

    if (button_pressed == false) {
        button_pressed= true;
        exti_set_trigger(EXTI10, EXTI_TRIGGER_RISING);
    } else {
        button_pressed= false;
        exti_set_trigger(EXTI10, EXTI_TRIGGER_FALLING);
    }
}

void encoder_button_init(void){
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_AFIO);

    //  Enable EXTI0 interrupt. 
    nvic_enable_irq(NVIC_EXTI15_10_IRQ);

    // Set GPIO0 (in GPIO port A) to 'input open-drain'. 
    gpio_set_mode(GPIOD, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO10);

    // Configure the EXTI subsystem. 
    exti_select_source(EXTI10, GPIOD);
    exti_set_trigger(EXTI10, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI10);
    
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
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOE);

	gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, CS|RST|RD);
	// gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED);
	// gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, 0xFF);

    fsmc_init();

	ui_init();
    while (1) {
        lv_task_handler();
        msleep(10);
    }

}

void fsmc_init(){
//  FSMC _BCRx, and FSMC_BTRx/FSMC_BWTRx
// MMIO32();

gpio_set_mode(GPIOD, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0|GPIO1|GPIO4|GPIO5|GPIO7|GPIO11|GPIO14|GPIO15);
gpio_set_mode(GPIOE, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO7| GPIO8 | GPIO9 | GPIO10);
// RCC_AHBENR |=  RCC_AHBENR_FSMCEN;
rcc_periph_clock_enable(RCC_FSMC);
#define FSMC_BCR1 (MMIO32(0xA0000000))
#define FSMC_BTR1 (MMIO32(0xA0000000+0x04))
#define FSMC_BWTR1 (MMIO32(0xA0000000+0x104))

//#define LCD_BASE    ((uint32_t)(0x60000000 | 0x0001fffE))



    FSMC_BWTR1 = 0x0FFFFFFF;
                                                                                 // BCR&BTR Register see RM000008 p. 541
    FSMC_BCR1 = 0 << 19  |       // FSMC_BCRx_CBURSTRW_Pos write 0 - async 1 - sycnc
                0 << 15 |       //  FSMC_BCRx_ASYNCWAIT_Pos Wait signal during asynchronous transfers
                0 << 14    |       // EXTMODExtended mode enable. Use BWTR register or no
                0 << 13    |       //  WAITEN Wait enable bit.
                1 << 12      |       // wren Write enable bit
                0 << 11   |       // WAITCFG Wait timing configuration. 0: NWAIT signal is active one data cycle before wait state 1: NWAIT signal is active during wait state
                0 << 10   |       // Wrapped burst mode support
                0 << 9   |       // Wait signal polarity bit. 0: NWAIT active low. 1: NWAIT active high
                0 << 8   |       // Burst enable bit
                1 << 6    |       // Flash access enable
                // 1 << 4      |       // 0 = 8b 1 = 16b
                2 << 2      |       // 0 = SRAM 1 = CRAM 2 = NOR
                0 << 1     |       // Multiplexing Address/Data
                1 << 0;            // Memory bank enable bit
 
 
    FSMC_BTR1 = 0 << 0  |    // Address setup phase duration 0..F * HCLK
                0 << 4  |    // Address-hold phase duration 1..F * 2 * HCLK
                1 << 8  |    // Data-phase duration 1..FF * 2 * HCLK
                0 << 16 |    // Bus turnaround phase duration 0...F
                1 << 20  |    // for FSMC_CLK signal 1 = HCLK/2, 2 = HCLK/3 ...  F= HCLK/16
                0 << 24  |    // Data latency for synchronous NOR Flash memory 0(2CK)...F(17CK)
                0 << 28;      // Access mode 0 = A, 1 = B, 2 = C, 3 = D Use w/EXTMOD bit
}