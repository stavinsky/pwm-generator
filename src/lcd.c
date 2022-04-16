#include "lcd.h"
#include "utils.h"
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include "ui.h"

int8_t fch, bcl, fcl, bch;
uint16_t c;
uint16_t test_data_out = 0x00;
#define  period 14 
#define timer_master TIM5
#define timer_slave TIM3
bool _transparent=false;
static void lcd_clear_xy(void);
void lcd_init()
{
    set_high(GPIOB, RD);
    set_high(GPIOB, RST);
    msleep(5);
    set_low(GPIOB, RST);
    msleep(15);
    set_high(GPIOB, RST);
    msleep(15);

    write_com_data(0x11,0x2004);
    write_com_data(0x13,0xCC00);
    write_com_data(0x15,0x2600);
    write_com_data(0x14,0x252A);
    write_com_data(0x12,0x0033);
    write_com_data(0x13,0xCC04);
    write_com_data(0x13,0xCC06);
    write_com_data(0x13,0xCC4F);
    write_com_data(0x13,0x674F);
    write_com_data(0x11,0x2003);
    write_com_data(0x30,0x2609);
    write_com_data(0x31,0x242C);
    write_com_data(0x32,0x1F23);
    write_com_data(0x33,0x2425);
    write_com_data(0x34,0x2226);
    write_com_data(0x35,0x2523);
    write_com_data(0x36,0x1C1A);
    write_com_data(0x37,0x131D);
    write_com_data(0x38,0x0B11);
    write_com_data(0x39,0x1210);
    write_com_data(0x3A,0x1315);
    write_com_data(0x3B,0x3619);
    write_com_data(0x3C,0x0D00);
    write_com_data(0x3D,0x000D);
    write_com_data(0x16,0x0007);
    write_com_data(0x02,0x0013);
    write_com_data(0x03,0x0003);
    write_com_data(0x01,0x0127);
    write_com_data(0x08,0x0303);
    write_com_data(0x0A,0x000B);
    write_com_data(0x0B,0x0003);
    write_com_data(0x0C,0x0000);
    write_com_data(0x41,0x0000);
    write_com_data(0x50,0x0000);
    write_com_data(0x60,0x0005);
    write_com_data(0x70,0x000B);
    write_com_data(0x71,0x0000);
    write_com_data(0x78,0x0000);
    write_com_data(0x7A,0x0000);
    write_com_data(0x79,0x0007);
    write_com_data(0x07,0x0051);
    write_com_data(0x07,0x0053);
    write_com_data(0x79,0x0000);

}
void lcd_set_color(uint8_t r, uint8_t g, uint8_t b)
{
    fch=((r&248)|g>>5);
    fcl=((g&28)<<3|b>>3);
}


inline void lcd_set_xy(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
    write_com_data(0x46,(x2 << 8) | x1);
    write_com_data(0x47,y2);
    write_com_data(0x48,y1);
    write_com_data(0x20,x1);
    write_com_data(0x21,y1);
}

static void lcd_clear_xy()
{
    lcd_set_xy(0,0,disp_size_x,disp_size_y);
}

void lcd_fill_rect(long pix)
{
    lcd_clear_xy();
	write_com(0x22);
	set_high(GPIOB, RS);

	for (int i =0 ; i<pix; i++){
            write_bus(fch, fcl);
  }
}

void lcd_set_pixel_color(unsigned int color)
{
    write_com(0x22);
    write_data((color>>8),(color&0xFF));    // rrrrrggggggbbbbb
}

static void draw_pixel(int x, int y)
{
    lcd_set_xy(x, y, x, y);
    lcd_set_pixel_color((fch<<8)|fcl);
    lcd_clear_xy();
}

void my_flush_cb(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p){

    lcd_set_xy(area->x1, area->y1, area->x2, area->y2);
    write_com(0x22);
    volatile uint32_t cnt =(area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) ;
    volatile uint16_t start = color_p->full;
    volatile uint16_t end = color_p->full;
    lv_color_t *end_p = color_p + (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) ;
    while ( color_p < end_p){
            write_bus_16(color_p->full);
            end = color_p->full;
            color_p++;
    }
    lv_disp_flush_ready(disp_drv);
}

void test_fill_display(uint16_t color) {
    uint32_t max = 76800-5;
    lcd_clear_xy();
    write_com(0x22);
    for(uint32_t i = 0; i<max; i++){
        write_bus_16(color);
    } 

}
// void tim5_isr(){
//     uint16_t data_to_transfer = dma_get_number_of_data(DMA1, DMA_CHANNEL3);
//     if (data_to_transfer > 0 ) {
//         tm_start(data_to_transfer);
//     }
//     TIM_SR(TIM5) &= ~TIM_SR_UIF;
// }
void dma1_channel3_isr() {
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL3, DMA_TCIF)){
        timer_set_counter(timer_slave, 0);
        lv_disp_flush_ready(&disp_drv);
        dma_disable_channel(DMA1, DMA_CHANNEL3);
    }
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL3, DMA_HTIF|DMA_TCIF|DMA_TEIF|DMA_GIF);
}
void tm_start(int times){
    timer_disable_counter(timer_slave);
    timer_set_oc_value(timer_master, TIM_OC4, times);
    // timer_set_counter(timer_slave, 0);
    timer_set_counter(timer_master, 0);
    timer_enable_counter(timer_slave);
}
void dma_timer_init() {
    rcc_periph_clock_enable(RCC_TIM5); 
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_reset_pulse(RST_TIM5);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM5_CH4);
    timer_set_mode(timer_master, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    // nvic_enable_irq(NVIC_TIM5_IRQ);


    timer_enable_oc_output(timer_master, TIM_OC4);
    timer_set_prescaler(timer_master, 0);
    timer_set_repetition_counter(timer_master, 0);
    timer_one_shot_mode(timer_master);
    timer_set_oc_polarity_high(timer_master,  TIM_OC4);
    timer_set_period(timer_master, 65535);
    timer_set_oc_value(timer_master, TIM_OC4, 65500);
    timer_set_counter(timer_master, 65501);
    timer_set_oc_mode(timer_master, TIM_OC4, TIM_OCM_PWM1);
    timer_set_master_mode(timer_master, TIM_CR2_MMS_COMPARE_OC4REF);
    timer_enable_irq(timer_master, TIM_DIER_UIE);
    timer_slave_set_trigger(timer_master, TIM_SMCR_TS_ITR1);
    timer_slave_set_mode(timer_master, TIM_SMCR_SMS_ECM1);

    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_TIM3); 
    rcc_periph_reset_pulse(RST_TIM3);
    timer_set_mode(timer_slave, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(timer_slave, 0);
    timer_set_repetition_counter(timer_slave, 0);
    timer_continuous_mode(timer_slave);
    timer_set_period(timer_slave, period - 1 );
    timer_set_oc_mode(timer_slave, TIM_OC2, TIM_OCM_PWM2);
    timer_set_oc_value(timer_slave, TIM_OC2, 10   );
    timer_disable_oc_output(timer_slave,     TIM_OC2);
    timer_set_oc_polarity_low(timer_slave,  TIM_OC2);
    timer_enable_oc_output(timer_slave, TIM_OC2);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5|GPIO1);

    timer_set_oc_mode(timer_slave, TIM_OC4, TIM_OCM_PWM2);
    timer_set_oc_value(timer_slave, TIM_OC4, 8);
    timer_disable_oc_output(timer_slave,     TIM_OC4);
    timer_set_oc_polarity_low(timer_slave,  TIM_OC4);
    timer_enable_oc_output(timer_slave, TIM_OC4);
    // timer_set_dma_on_compare_event(timer_slave);

    timer_slave_set_mode(timer_slave, TIM_SMCR_SMS_GM);
    timer_slave_set_trigger(timer_slave, TIM_SMCR_TS_ITR2);
    // timer_set_dma_on_update_event(timer_slave);
    // timer_update_on_any(timer_slave);
    // timer_update_on_overflow(timer_slave);
    timer_enable_irq(TIM3, TIM_DIER_CC4DE);
    timer_set_master_mode(timer_slave,TIM_CR2_MMS_UPDATE);
    timer_enable_counter(timer_slave);
    timer_enable_counter(timer_master);
}


void dma_init(){
    nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ);
    rcc_periph_clock_enable(RCC_DMA1);
    dma_set_memory_size(DMA1, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT);
	dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL3); 
    dma_set_read_from_memory(DMA1, DMA_CHANNEL3);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL3);
    // dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL3);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL3);
    // dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL3);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL3, (uint32_t)&GPIO_ODR(GPIOD));
    dma_set_priority(DMA1, DMA_CHANNEL3, DMA_CCR_PL_VERY_HIGH);
}


void my_flush_cb2(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p){
    uint32_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) * 2;
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, WR);
    lcd_set_xy(area->x1, area->y1, area->x2, area->y2);
    write_com(0x22);

    dma_set_memory_address(DMA1, DMA_CHANNEL3, (uint32_t)color_p);
    dma_set_number_of_data(DMA1, DMA_CHANNEL3, size);


    GPIO_ODR(GPIOD) = color_p->full >> 8;
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, WR);
    // timer_generate_event(TIM3, TIM_EGR_CC4G);
    dma_enable_channel(DMA1, DMA_CHANNEL3);
    tm_start(size);
}