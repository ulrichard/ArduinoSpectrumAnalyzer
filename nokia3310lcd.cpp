/*
 * Name         :  nokia3310lcd.cpp
 * Description  :  This is a driver for the PCD8544 graphic LCD.
 *                 Based on the code written by Fandi Gunawan and Sylvain Bissonette
 *                 This driver is buffered in 504 uint8_ts memory be sure
 *                 that your MCU has bigger memory
 * Author       :  Richard Ulrich <richi@paraeasy.ch>
 * License      :  GPL v. 3
*/

#include "nokia3310lcd.h"
#include "Spi.h"
// arduino
#include <WProgram.h>
// stdlib
#include <stdio.h>
//#include <avr/io.h>
//#include <string.h>
//#include <avr/pgmspace.h>
//#include <avr/interrupt.h>


Nokia3310LCD::Nokia3310LCD(const uint8_t pinCmdDtaSw, const uint8_t pinReset, const uint8_t pinChipSel)
  : pinCmdDtaSw_(pinCmdDtaSw), pinReset_(pinReset), pinChipSel_(pinChipSel)
{

}

Nokia3310LCD::~Nokia3310LCD()
{
  
}

void Nokia3310LCD::init()
{
    pinMode(pinCmdDtaSw_, OUTPUT); // command / data switch
    pinMode(pinReset_,    OUTPUT); // active low reset   
    pinMode(pinChipSel_,  OUTPUT); // chip select  

    // Pull-up on reset pin.
    digitalWrite(pinReset_, HIGH);
    delay(20);
    digitalWrite(pinReset_, LOW);
    delay(20);
    digitalWrite(pinReset_, HIGH);

    // Disable LCD controller
    digitalWrite(pinChipSel_, HIGH);

    LcdSend(0x21, LCD_CMD); // LCD Extended Commands.
    LcdSend(0xC8, LCD_CMD); // Set LCD Vop (Contrast).
    LcdSend(0x06, LCD_CMD); // Set Temp coefficent.
    LcdSend(0x13, LCD_CMD); // LCD bias mode 1:48.
    LcdSend(0x20, LCD_CMD); // LCD Standard Commands,Horizontal addressing mode
    LcdSend(0x0C, LCD_CMD); // LCD in normal mode.

    // Clear display on first time use
    LcdClear();
    LcdUpdate();
    
    LcdContrast(0x7F);
}

/** @brief  Clears the display. LcdUpdate must be called next. */
void Nokia3310LCD::LcdClear(void)
{
    memset(LcdCache, 0x00, LCD_CACHE_SIZE); 
    
    // Reset watermark pointers to full
    LoWaterMark = 0;
    HiWaterMark = LCD_CACHE_SIZE - 1;

    UpdateLcd = true;
}

/** @brief : Set display contrast.
 *  @param uint8_t contrast Contrast value from 0x00 to 0x7F.
 */
void Nokia3310LCD::LcdContrast(uint8_t contrast)
{
    // LCD Extended Commands.
    LcdSend(0x21, LCD_CMD);

    // Set LCD contrast level.
    LcdSend(0x80 | contrast, LCD_CMD);

    // LCD Standard Commands, horizontal addressing mode.
    LcdSend(0x20, LCD_CMD);
}

/** @brief Sets cursor location to xy location corresponding to basic font size.
 *  @param uint8_t x, y -> Coordinate for new cursor position. Range: 1,1 .. 14,6
 */
Nokia3310LCD::RETVAL Nokia3310LCD::LcdGotoXYFont(uint8_t x, uint8_t y)
{
    // Boundary check, slow down the speed but will guarantee this code wont fail
    if(x > 14)
        return OUT_OF_BORDER;
    if(y > 6)
        return OUT_OF_BORDER;
        
    // Calculate index. It is defined as address within 504 uint8_ts memory 
    LcdCacheIdx = (x - 1) * 6 + (y - 1) * 84;
    return OK;
}

/** @brief  Displays a character at current cursor location and increment cursor location.
 *  @param  uint8_t ch   Character to write.
 */
Nokia3310LCD::RETVAL Nokia3310LCD::LcdChr(uint8_t ch)
{
    if(LcdCacheIdx < LoWaterMark)
        LoWaterMark = LcdCacheIdx;

    if((ch < 0x20) || (ch > 0x7b))
        ch = 92; // Convert to a printable character.

    for(uint8_t i = 0; i < 5; i++)
    {
        // Copy lookup table from Flash ROM to LcdCache
        LcdCache[LcdCacheIdx++] = pgm_read_byte(&(FontLookup[ch - 32][i])) << 1;
    }
    
    if(LcdCacheIdx > HiWaterMark)
        HiWaterMark = LcdCacheIdx;

    // Horizontal gap between characters.
    LcdCache[LcdCacheIdx] = 0x00;
    
    // At index number LCD_CACHE_SIZE - 1, wrap to 0
    if(LcdCacheIdx == (LCD_CACHE_SIZE - 1))
    {
        LcdCacheIdx = 0;
        return OK_WITH_WRAP;
    }
    
    // Otherwise just increment the index 
    LcdCacheIdx++;
    return OK;
}

/** @brief  Displays a character at current cursor location and increment cursor location according to font size. This function is dedicated to print string laid in SRAM
 *  @param  uint8_t* dataArray   Array contained string of char to be written into cache.
 */
Nokia3310LCD::RETVAL Nokia3310LCD::LcdStr(const char* dataArray)
{
    for(uint8_t tmpIdx = 0; dataArray[tmpIdx] != '\0'; ++tmpIdx)
        if(LcdChr(dataArray[tmpIdx]) == OUT_OF_BORDER)
            return OUT_OF_BORDER;
    
    return OK;
}

/** @brief  Displays a pixel at given absolute (x, y) location.
 *  @param  uint8_t         x, y  Absolute pixel coordinates
 *  @param  LcdPixelMode mode  Off, On or Xor
 */
Nokia3310LCD::RETVAL Nokia3310LCD::LcdPixel(uint8_t x, uint8_t y, LcdPixelMode mode)
{
    // Prevent from getting out of border
    if(x > LCD_X_RES)
        return OUT_OF_BORDER;
    if(y > LCD_Y_RES) 
        return OUT_OF_BORDER;

    // Recalculating index and offset
    word index  = ((y / 8) * LCD_X_RES) + x;
    uint8_t offset = y - ((y / 8) * 8);
    uint8_t data = LcdCache[index];

    // Bit processing
    switch(mode)
    {
    case PIXEL_OFF: // Clear mode
        data &= (~(0x01 << offset));
        break;
    case PIXEL_ON:  // On mode
        data |= (0x01 << offset);
        break;
    case PIXEL_XOR: // Xor mode
        data ^= (0x01 << offset);
        break;
    }

    // Final result copied to cache
    LcdCache[index] = data;

    if(index < LoWaterMark)
        LoWaterMark = index;

    if(index > HiWaterMark)
        HiWaterMark = index;

    return OK;
}

/** @brief  Draws a line between two points on the display.
 *  @param uint8_t  x1, y1   Absolute pixel coordinates for line origin.
 *  @param uint8_t  x2, y2   Absolute pixel coordinates for line end.
 */
Nokia3310LCD::RETVAL Nokia3310LCD::LcdLine(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, LcdPixelMode mode)
{
    // Calculate differential form
    // dy   y2 - y1
    // -- = -------
    // dx   x2 - x1

    // Take differences
    int dy = y2 - y1;
    int dx = x2 - x1;

    int stepy = 1;
    if(dy < 0)
    {
        dy    = -dy;
        stepy = -1;
    }

    int stepx = 1;
    if(dx < 0)
    {
        dx    = -dx;
        stepx = -1;
    }

    dx <<= 1;
    dy <<= 1;

    // Draw initial position
    RETVAL response = LcdPixel(x1, y1, mode);
    if(response)
        return response;

    // Draw next positions until end
    if(dx > dy)
    {
        // Take fraction
        int fraction = dy - (dx >> 1);
        while(x1 != x2)
        {
            if(fraction >= 0)
            {
                y1 += stepy;
                fraction -= dx;
            }
            x1 += stepx;
            fraction += dy;

            // Draw calculated point
            response = LcdPixel(x1, y1, mode);
            if(response)
                return response;

        }
    }
    else // dx <= dy
    {
        // Take fraction
        int fraction = dx - (dy >> 1);
        while(y1 != y2)
        {
            if(fraction >= 0)
            {
                x1 += stepx;
                fraction -= dy;
            }
            y1 += stepy;
            fraction += dx;

            // Draw calculated point
            response = LcdPixel(x1, y1, mode);
            if(response)
                return response;
        }
    }

    // Set update flag to be true
    UpdateLcd = true;
    return OK;
}

/** @brief  Image mode display routine. */
void Nokia3310LCD::LcdImage(const uint8_t *imageData)
{
    memcpy_P(LcdCache, imageData, LCD_CACHE_SIZE);
    // Reset watermark pointers to be full
    LoWaterMark = 0;
    HiWaterMark = LCD_CACHE_SIZE - 1;

    UpdateLcd = true;
}

/** @brief Copies the LCD cache into the device RAM. */
void Nokia3310LCD::LcdUpdate(void)
{
    LoWaterMark = min(LCD_CACHE_SIZE - 1, LoWaterMark);
    HiWaterMark = min(LCD_CACHE_SIZE - 1, HiWaterMark);
    
    //  Set base address according to LoWaterMark.
    LcdSend(0x80 | (LoWaterMark % LCD_X_RES), LCD_CMD);
    LcdSend(0x40 | (LoWaterMark / LCD_X_RES), LCD_CMD);
//    delay(1);

    //  Serialize the display buffer.
    for(uint16_t i = LoWaterMark; i <= HiWaterMark; ++i)
        LcdSend(LcdCache[i], LCD_DATA);

    //  Reset watermark pointers.
    LoWaterMark = LCD_CACHE_SIZE - 1;
    HiWaterMark = 0;

    // Set update flag to be true
    UpdateLcd = false;
}

/** @brief Copies the LCD cache into the device RAM. */
void Nokia3310LCD::LcdFullUpdate(uint8_t steps)
{
    steps = max(1, min(8, steps));
    const size_t step_size = LCD_CACHE_SIZE / steps;
        
    for(uint8_t i=0; i<steps; ++i)
    {
        LoWaterMark = i * step_size;
        HiWaterMark = LoWaterMark + step_size; 
        UpdateLcd = true;   
        LcdUpdate();
    }
}

/** @brief  Sends data to display controller. */
void Nokia3310LCD::LcdSend(uint8_t data, LcdCmdData cd)
{
    //  Enable display controller (active low).
    digitalWrite(pinChipSel_, LOW);

    digitalWrite(pinCmdDtaSw_, cd == LCD_DATA);

    //  Send data to display controller.
    Spi.transfer(data);

    // Disable display controller.
    digitalWrite(pinChipSel_, HIGH);
}

