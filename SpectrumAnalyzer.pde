/*
  Testing the nokia lcd
 
   The following pins are used:
 *   13  spi sclk -> serial clock
 *   12  spi miso -> serial data from CYWM9635 to arduino 
 *   11  spi mosi -> serial data from arduino to LCD and CYWM9635
 *   10  spi ss   -> 
 *    9  LCD command or data 
 *    8  LCD reset      
 *    7  LCD chip select 
 *    6  mode button
 *    5  CYW reset
 *    4  CYW nSS slave select
 */
#include "nokia3310lcd.h"
#include "CYWM6935.h"
// std lib
#include <ctype.h>


Nokia3310LCD  disp(9, 8, 7);
CYWM6935      radio(5, 4);
uint8_t       signal [Nokia3310LCD::LCD_X_RES + 1];
uint8_t       sigmax [Nokia3310LCD::LCD_X_RES + 1];
uint32_t      sigsum [Nokia3310LCD::LCD_X_RES + 1];
uint8_t       maxhist[Nokia3310LCD::LCD_X_RES + 1];
uint8_t       maxhistpos;
static const uint8_t  NORMAL_STEP_DECREASE = 5;   // by how much to decrease in each step while in normal mode
static const uint8_t  SLOW_STEP_DECREASE   = 1;   // by how much to decrease in each step while in slow mode
static const uint16_t SUMM_MEAS_COUNT      = 300; // summing the last n measures to get an average
static const uint8_t  pinBtnMode           = 6;
uint32_t lastUpdate;
uint32_t lastCount; 

enum OperationMode
{
    MOD_COMBINED,
    MOD_SLOW,
    MOD_FAST,
    MOD_ADDING,
    MOD_HISORY,
    
    MOD_OPTIONS_COUNT
} opmode;
static const char ModNames[5][8] = {"Combine", "Slow   ", "Fast   ", "Adding ", "History"};

bool DebouncedBtnPress(const uint8_t pinBtn, const uint16_t debounceDelay = 50);

void setup()   
{
    opmode = MOD_COMBINED;
    maxhistpos = 0;
    memset(signal,  0, sizeof(signal));
    memset(sigmax,  0, sizeof(sigmax));
    memset(sigsum,  0, sizeof(sigsum));
    memset(maxhist, 0, sizeof(maxhist));
        
    Serial.begin(19200);  
    Serial.println("init");
    
    pinMode(pinBtnMode,    INPUT);
    
    Spi.init();
    Spi.mode((1 << SPR0) | (1 << SPR1));  // set the spi clock to 125kHz
    
    disp.init();
    radio.init();    
    disp.LcdClear();
    
    lastUpdate = 0;
    lastCount  = 0;
}

void loop()                     
{    
    static uint32_t loopcount = 0;
    const uint32_t loopStart = millis();
  
    // Read signals and process according "adding" and "fast"
    uint8_t maxsig = 0;
    const uint8_t decrStep = (opmode == MOD_SLOW ? SLOW_STEP_DECREASE : NORMAL_STEP_DECREASE);
    for(uint8_t n=0; n<disp.LCD_X_RES - 1; ++n)
    {
        const uint8_t s = radio.RSSI(n); // returns  0 - 31
    
        if(opmode == MOD_FAST)
            signal[n] = s;
        else if(s > signal[n] - decrStep) 
	    signal[n] = s;     
        else  if(signal[n] !=0) // if not greater, decrease it
            signal[n] -= decrStep;
            
        sigmax[n] = max(s, sigmax[n]);
        
        sigsum[n] = sigsum[n] - sigsum[n] / min(loopcount, SUMM_MEAS_COUNT) + s;
        
        maxsig = max(maxsig, s);
    }  
    maxhist[maxhistpos] = maxsig;
    if(++maxhistpos > disp.LCD_X_RES)
        maxhistpos = 0;

  
    // Draw the signal array
    static const uint8_t nullpos = 39;
    uint8_t *currarr = (opmode == MOD_ADDING ? sigmax : (opmode == MOD_HISORY ? maxhist : signal));
    for(uint8_t n=0; n<disp.LCD_X_RES-1; ++n)
    {
        // show the current value as a line
        const uint8_t sval = currarr[opmode == MOD_HISORY ? (n + maxhistpos) % disp.LCD_X_RES : n];
	disp.LcdLine(n, n,       8, nullpos,        Nokia3310LCD::PIXEL_OFF);
	disp.LcdLine(n, n, nullpos, nullpos - sval, Nokia3310LCD::PIXEL_ON);

        if(opmode == MOD_COMBINED)
        {
            // show the max as a pixel
            if(sigmax[n])
                disp.LcdPixel(n, nullpos - sigmax[n], Nokia3310LCD::PIXEL_ON);
            // show the avg as an alternating pixel
            if(loopcount % 2)
            {
                const uint8_t avgval = sigsum[n] / min(loopcount, SUMM_MEAS_COUNT);
                if(avgval > 5)
                    disp.LcdPixel(n, nullpos - avgval, Nokia3310LCD::PIXEL_XOR);                    
            }
        }
    }  
    
    if(!(loopcount % 20))
    {
        static uint8_t hdrcounter = 0;
        static const char strings[3][15] = {"ICM Spectrum  ", "Analyzer      ", "2.40-2.484 GHz"};
        
        disp.LcdGotoXYFont(1, 1);
        disp.LcdStr(strings[hdrcounter++ % 3]);
    }
    
    // Set lower line acording to actual mode
    disp.LcdGotoXYFont(1, 6);
    disp.LcdStr("Mode ");
    disp.LcdStr(ModNames[opmode]);

    // display the sweeps per second
    if(millis() - lastUpdate > 3000)
    {
        disp.LcdGotoXYFont(14, 6);
        char tmp[5];
        sprintf(tmp, "%d", 1000 * (loopcount - lastCount) / (millis() - lastUpdate));
        disp.LcdStr(tmp);
        
        lastUpdate = millis();
        lastCount  = loopcount;
    }
   
    disp.LcdUpdate();   
   
    // change mode from push button
    if(DebouncedBtnPress(pinBtnMode))
    {
        switch(opmode)
        {
        case MOD_COMBINED:
            opmode = MOD_SLOW;
            break;
        case MOD_SLOW:
            opmode = MOD_FAST;
            break;
        case MOD_FAST:
            opmode = MOD_ADDING;
            break;
        case MOD_ADDING:
            opmode = MOD_HISORY;
            break;
        case MOD_HISORY:
        default:
            opmode = MOD_COMBINED;
            break;
        }      
    }
   
    
    // change mode from serial command
    if(Serial.available()) 
    {
        const char cc = toupper(Serial.read());
        switch(cc)
        {
        case 'C':
            opmode = MOD_COMBINED;
            break;
        case 'S':
            opmode = MOD_SLOW;
            break;
        case 'F':
            opmode = MOD_FAST;
            break;
        case 'A':
            opmode = MOD_ADDING;
            break;
        case 'H':
            opmode = MOD_HISORY;
            break;
        case 'R':
            memset(signal,  0, sizeof(signal));
            memset(sigmax,  0, sizeof(sigmax));
            memset(sigsum,  0, sizeof(sigsum));
            memset(maxhist, 0, sizeof(maxhist));
            break;
        default:
            Serial.println("\nAvailable modes are:");
            Serial.println("C : medium decrease rate including MAX and AVG");
            Serial.println("S : slow decrease rate");
            Serial.println("F : realtime signal");
            Serial.println("A : exposure mode");
            Serial.println("H : history of max signal strength over time");
            Serial.println("R : reset the accumulated values");
            break;
        }
    }
    
    ++loopcount;
}

bool DebouncedBtnPress(const uint8_t pinBtn, const uint16_t debounceDelay)
{
    static bool btnState = false;
    static bool btnLastState = false;
    static uint32_t lastDebounceTime = 0;
    const bool btnReading = (digitalRead(pinBtn) == HIGH);
//    Serial.println(btnReading ? "btn pressed" : "btn open");
    if(btnReading != btnLastState) 
        lastDebounceTime = millis(); // reset the debouncing timer
    if((millis() - lastDebounceTime) > debounceDelay)
        btnState = btnReading;
    btnLastState = btnReading;
    return (btnState && !btnLastState);
} 
