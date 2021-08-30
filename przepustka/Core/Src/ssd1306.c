/*
 * ssd1306.c
 *
 *  Created on: 30 sie 2021
 *      Author: Mariusz
 */

#include "main.h"
#include "ssd1306.h"
extern uint32_t  time;
// Screenbuffer
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Screen object
static SSD1306_t SSD1306;


static uint8_t ssd1306_WriteCommand(SSD1306_t *oled, uint8_t command)
{ uint8_t result;


	result=HAL_I2C_Mem_Write(oled->oled_i2c, oled->Address, 0x00, 1, &command, 1, 10);
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,1);
    return result;
}


uint8_t ssd1306_Init(SSD1306_t *oled, I2C_HandleTypeDef *i2c, uint8_t Address)
{ // Wait for the screen to boot
    oled->oled_i2c=i2c;
    oled->Address=Address<<1;
	HAL_Delay(100);
    int status = 0;

    // Init LCD
    status += ssd1306_WriteCommand(oled, 0xAE);   // Display off
    status += ssd1306_WriteCommand(oled, 0x20);   // Set Memory Addressing Mode
    status += ssd1306_WriteCommand(oled, 0x00);   // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    status += ssd1306_WriteCommand(oled, 0xB0);   // Set Page Start Address for Page Addressing Mode,0-7
    status += ssd1306_WriteCommand(oled, 0xC8);   // Set COM Output Scan Direction
    status += ssd1306_WriteCommand(oled, 0x00);   // Set low column address
    status += ssd1306_WriteCommand(oled, 0x10);   // Set high column address
    status += ssd1306_WriteCommand(oled, 0x40);   // Set start line address
    status += ssd1306_WriteCommand(oled, 0x81);   // set contrast control register
    status += ssd1306_WriteCommand(oled, 0xFF);
    status += ssd1306_WriteCommand(oled, 0xA1);   // Set segment re-map 0 to 127
    status += ssd1306_WriteCommand(oled, 0xA6);   // Set normal display

    status += ssd1306_WriteCommand(oled, 0xA8);   // Set multiplex ratio(1 to 64)
    status += ssd1306_WriteCommand(oled, SSD1306_HEIGHT - 1);

    status += ssd1306_WriteCommand(oled, 0xA4);   // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    status += ssd1306_WriteCommand(oled, 0xD3);   // Set display offset
    status += ssd1306_WriteCommand(oled, 0x00);   // No offset
    status += ssd1306_WriteCommand(oled, 0xD5);   // Set display clock divide ratio/oscillator frequency
    status += ssd1306_WriteCommand(oled, 0x80);   // Set divide ratio
    status += ssd1306_WriteCommand(oled, 0xD9);   // Set pre-charge period
    status += ssd1306_WriteCommand(oled, 0x22);

    status += ssd1306_WriteCommand(oled, 0xDA);   // Set com pins hardware configuration
#ifdef SSD1306_COM_LR_REMAP
    status += ssd1306_WriteCommand(oled, 0x32);   // Enable COM left/right remap
#else
    status += ssd1306_WriteCommand(oled, 0x12);   // Do not use COM left/right remap
#endif // SSD1306_COM_LR_REMAP

    status += ssd1306_WriteCommand(oled, 0xDB);   // Set vcomh
    status += ssd1306_WriteCommand(oled, 0x20);   // 0x20,0.77xVcc
    status += ssd1306_WriteCommand(oled, 0x8D);   // Set DC-DC enable
    status += ssd1306_WriteCommand(oled, 0x14);   //
    status += ssd1306_WriteCommand(oled, 0xAF);   // Turn on SSD1306 panel

    if (status != 0) {
        return 1;
    }

    // Clear screen
    ssd1306_Fill(White);

    // Flush buffer to screen
    ssd1306_UpdateScreen(oled);

    // Set default values for screen object
    SSD1306.CurrentX = 0;
    SSD1306.CurrentY = 0;

    SSD1306.Initialized = 1;

    return 0;


}


    //
    //  Fill the whole screen with the given color
    //
    void ssd1306_Fill(SSD1306_COLOR color)
    {
        // Fill screenbuffer with a constant value (color)
        uint32_t i;

        for(i = 0; i < sizeof(SSD1306_Buffer); i++)
        {
            SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
        }
    }

    //
    //  Write the screenbuffer with changed to the screen
    //
    void ssd1306_UpdateScreen(SSD1306_t *oled)
    { uint32_t time1;
        uint8_t i;
        time1=HAL_GetTick();
//           	 ssd1306_WriteCommand(oled, 0xB0);
//        	 ssd1306_WriteCommand(oled, 0x00);
//        	 ssd1306_WriteCommand(oled, 0x10);
        if ((oled->oled_i2c->hdmatx->State ==HAL_DMA_STATE_READY)&&(oled->oled_i2c->State==HAL_I2C_STATE_READY))
        {
    	 ssd1306_WriteCommand(oled, 0x22);
     	 ssd1306_WriteCommand(oled, 0x0);
     	 ssd1306_WriteCommand(oled, 0xFF);
     	 ssd1306_WriteCommand(oled, 0x21);
     	 ssd1306_WriteCommand(oled, 0x0);
     	 ssd1306_WriteCommand(oled, 127);
       //  HAL_I2C_Mem_Write(oled->oled_i2c, oled->Address, 0x40, 1, &SSD1306_Buffer[0], SSD1306_WIDTH*8,150);
            HAL_I2C_Mem_Write_DMA(oled->oled_i2c, oled->Address, 0x40, 1, &SSD1306_Buffer[0], SSD1306_WIDTH*8);
        }
            time=HAL_GetTick()-time1;


    }

    //
    //  Draw one pixel in the screenbuffer
    //  X => X Coordinate
    //  Y => Y Coordinate
    //  color => Pixel color
    //
    void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
    {
        if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
        {
            // Don't write outside the buffer
            return;
        }

        // Check if pixel should be inverted
        if (SSD1306.Inverted)
        {
            color = (SSD1306_COLOR)!color;
        }

        // Draw in the correct color
        if (color == White)
        {
            SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
        }
        else
        {
            SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
        }
    }


    //
    //  Draw 1 char to the screen buffer
    //  ch      => Character to write
    //  Font    => Font to use
    //  color   => Black or White
    //
    char ssd1306_WriteChar(char ch, FontDef Font, SSD1306_COLOR color)
    {
        uint32_t i, b, j;

        // Check remaining space on current line
        if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.FontWidth) ||
            SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.FontHeight))
        {
            // Not enough space on current line
            return 0;
        }

        // Translate font to screenbuffer
        for (i = 0; i < Font.FontHeight; i++)
        {
            b = Font.data[(ch - 32) * Font.FontHeight + i];
            for (j = 0; j < Font.FontWidth; j++)
            {
                if ((b << j) & 0x8000)
                {
                    ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR) color);
                }
                else
                {
                    ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
                }
            }
        }

        // The current space is now taken
        SSD1306.CurrentX += Font.FontWidth;

        // Return written char for validation
        return ch;
    }

    //
    //  Write full string to screenbuffer
    //
    char ssd1306_WriteString(char* str, FontDef Font, SSD1306_COLOR color)
    {
        // Write until null-byte
        while (*str)
        {
            if (ssd1306_WriteChar(*str, Font, color) != *str)
            {
                // Char could not be written
                return *str;
            }

            // Next char
            str++;
        }

        // Everything ok
        return *str;
    }

    //
    //  Invert background/foreground colors
    //
    void ssd1306_InvertColors(void)
    {
        SSD1306.Inverted = !SSD1306.Inverted;
    }

    //
    //  Set cursor position
    //
    void ssd1306_SetCursor(uint8_t x, uint8_t y)
    {
        SSD1306.CurrentX = x;
        SSD1306.CurrentY = y;
    }
