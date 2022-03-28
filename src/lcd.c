#include "lcd.h"
#include "utils.h"

int8_t fch, bcl, fcl, bch;

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
    lv_color_t *end_p = color_p + (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1) ;
    while ( color_p < end_p){
            write_bus_16(color_p->full);
            color_p++;
    }
    lv_disp_flush_ready(disp_drv);
}
