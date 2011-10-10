/*
 * Name         :  CYWM6935.cpp
 * Description  :  This is a partial driver for the CYWM6935 wireless usb chip.
 *                 It is only concerned with reading the RSSI value for different frequencies.
 *                 Based on the code written by Miguel A. Vallejo and Jason Hecker
 * Author       :  Richard Ulrich <richi@paraeasy.ch>
 * License      :  GPL v. 3
*/

#include "CYWM6935.h"
// arduino
#include <WProgram.h>
#include <util/delay.h>
// stdlib
#include <stdio.h>


CYWM6935::CYWM6935(const uint8_t pinReset, const uint8_t pinChipSel)
  : pinReset_(pinReset), pinChipSel_(pinChipSel)
{

}

CYWM6935::~CYWM6935()
{
  
}

void CYWM6935::init()
{
    pinMode(pinReset_,    OUTPUT); // active low reset   
    pinMode(pinChipSel_,  OUTPUT); // chip select  
    
    // reset the radio
    digitalWrite(pinReset_,   HIGH);
    digitalWrite(pinChipSel_, HIGH);
    delay(1);
    
    // initialize the radio module
    Write(REG_CLOCK_MANUAL, 0x41);
    Write(REG_CLOCK_ENABLE, 0x41);
    Write(REG_ANALOG_CTL,   0x44);	
    Write(REG_CRYSTAL_ADJ,  0x40);
    Write(REG_VCO_CAL,      0xC0);
}

const uint8_t CYWM6935::Read(const RADIO_REGISTERS address) const
{    
    digitalWrite(pinChipSel_, LOW);         // Enable module    
    Spi.transfer(address);                  // Send address    
    const uint8_t val = Spi.transfer(0x00); // Receive data    
    digitalWrite(pinChipSel_, HIGH);        // Disable module
    
    return val;
}

void CYWM6935::Write(const RADIO_REGISTERS address, const uint8_t value) const
{    
    digitalWrite(pinChipSel_, LOW);  // Enable module    
    Spi.transfer(0x80 | address);    // Send data
    Spi.transfer(value);             // Send data
    digitalWrite(pinChipSel_, HIGH); // Disable module
}

// Returns RSSI (0..31) for given channel
const uint8_t CYWM6935::RSSI(const uint8_t channel) const
{    
    Write(REG_CHANNEL, channel);       // Set channel    
    Write(REG_CONTROL, 0x80);          // Turn receiver on
    // Wait for receiver to start-up
    // SYNTH_SETTLE (200) + RECEIVER_READY (35) + RSSI_ADC_CONVERSION (50)
    delayMicroseconds(285);
    
    uint8_t value = 0;
    while(true)
    {
        // Force conversion
	Write(REG_CARRIER_DETECT, 0x00);
	Write(REG_CARRIER_DETECT, 0x80);
	
	delayMicroseconds(50);   // RSSI_ADC_CONVERSION (50)	
	value = Read(REG_RSSI);  // Read RSSI	
	if(value & 0x20)         // Exit if valid
            break;
    }

    Write(REG_CONTROL,0x00);     // Turn receiver off    
    
    return (value & 0x1F);       // Return lower 5 bits
}


    

