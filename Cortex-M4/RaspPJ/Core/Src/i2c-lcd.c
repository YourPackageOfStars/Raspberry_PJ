

#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly
extern void us_delay(uint16_t time);


#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup 4E or 3F 27

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for (int i=0; i<70; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	// 4 bit initialisation
	us_delay(50000);  // wait for >40ms
	lcd_send_cmd (0x30);
	us_delay(50000);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	us_delay(1000);  // wait for >100us
	lcd_send_cmd (0x30);
	us_delay(10000);
	lcd_send_cmd (0x20);  // 4bit mode
	us_delay(10000);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	us_delay(1000);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	us_delay(1000);
	lcd_send_cmd (0x01);  // clear display
	us_delay(1000);
	us_delay(1000);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	us_delay(1000);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
