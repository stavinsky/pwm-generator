#include "lcd.h"
#include "utils.h"

int8_t fch, bcl, fcl, bch;

bool _transparent=false;
static inline void write_bus_8(uint8_t VL){
    /*uint8_t mask = 0xFF;*/
    /*GPIOA->ODR = (GPIOA->ODR & ~mask ) | ( VL & mask);*/
    GPIOD_BSRR  = VL | 0x00FF0000;
    pulse_low(GPIOB, WR);
    /*GPIOA->ODR = VL;*/
    /*pulse_low(GPIOB->ODR, WR);*/
}

static inline void  write_bus(uint8_t VH, uint8_t VL){
    write_bus_8(VH);
    write_bus_8(VL);

}
static inline void write_com(uint8_t VL)
{
    set_low(GPIOB, RS);
    write_bus(0x00,VL);
    set_high(GPIOB, RS);
}

static inline void write_data(uint8_t VH, uint8_t VL)
{
    //set_high(GPIOB->ODR, RS);
    write_bus(VH, VL);
}

static inline void write_com_data(uint8_t com, uint16_t data)
{
    write_com(com);
    write_data(data>>8, data & 0x00FF);
}

/*static inline uint16_t read_bus(){*/
/*    uint8_t high=0;*/
/*    uint8_t low=0;*/
/*    set_low(GPIOB, RD);*/
/*    high = GPIOA_IDR & 0x00FF;*/
/*    set_high(GPIOB, RD);*/
/*    set_low(GPIOB, RD);*/
/*    low = GPIOA_IDR & 0x00FF;*/
/*    set_high(GPIOB, RD);*/
/*    set_low(GPIOB, RD);*/
/*    return (high << 8) | low;*/

/*}*/
/*static inline uint16_t read_data(){*/
/*    uint16_t r = 0;*/
/*    GPIOA_ODR = 0x00;*/
/*    GPIOA_CRL = 0x88888888;*/
/*    set_high(GPIOB, RS);*/
/*    set_high(GPIOB, WR);*/
/*    r = read_bus();*/
/*    GPIOA_CRL = 0x33333333;*/
/*    return r;*/
/*}*/

/*static inline uint16_t read_com_data(uint8_t com)*/
/*{*/
/*    uint16_t data =0 ;*/
/*    write_com(com);*/
/*    data = read_data();*/
/*    return data;*/
/*}*/


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


void inline lcd_set_xy(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2){
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
	write_com(0x22);
	/*set_high(GPIOB, RD);*/
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

    uint16_t x, y;
    uint16_t vh, vl;
    lcd_set_xy(area->x1, area->y1, area->x2, area->y2);
    write_com(0x22);
    /*set_high(GPIOB, RD);*/
    set_high(GPIOB, RS);
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            vh = (color_p->full) >> 8;
            vl = (color_p->full) & 0x00FF;
            write_bus(vh,vl);
            color_p++;
        }
    }
    lv_disp_flush_ready(disp_drv);
}
