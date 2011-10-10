/*
 * Description  :  This is a partial driver for the CYWM6935 wireless usb chip.
 *                 It is only concerned with reading the RSSI value for different frequencies.
 *                 13  sclk -> serial clock
 *                 12  miso -> 
 *                 11  mosi -> serial data from arduino to LCD
 *                 10  ss   -> 
 *                  5  CYW reset             (can be chosen)
 *                  4  CYW nSS slave select  (can be chosen)
 *
 * Author       :  Richard Ulrich <richi@paraeasy.ch>
 *
 * Credit       :  Miguel A. Vallejo and Jason Hecker
 * License      :  GPL v. 3
 */

#ifndef _CYWM6935_H_
#define _CYWM6935_H_

#include "Spi.h"
#include <avr/pgmspace.h>
#include <stdint.h>


class CYWM6935
{
public:
    CYWM6935(const uint8_t pinReset = 5, const uint8_t pinChipSel = 4);
    ~CYWM6935();
    
    void init();
    const uint8_t RSSI(const uint8_t channel) const;

private:    
    enum RADIO_REGISTERS
    {
       REG_ID               = 0x00,
       REG_CONTROL          = 0x03,
       REG_DATA_RATE        = 0x04,
       REG_CONFIG           = 0x05,
       REG_SERDES_CTL       = 0x06,
       REG_RX_INT_EN        = 0x07,
       REG_RX_INT_STAT      = 0x08,
       REG_RX_DATA_A        = 0x09,
       REG_RX_VALID_A       = 0x0A,
       REG_RX_DATA_B        = 0x0B,
       REG_RX_VALID_B       = 0x0C,
       REG_TX_INT_EN        = 0x0D,
       REG_TX_INT_STAT      = 0x0E,
       REG_TX_DATA          = 0x0F,
       REG_TX_VALID         = 0x10,
       REG_THRESHOLD_L      = 0x19,
       REG_THRESHOLD_H      = 0x1A,
       REG_WAKE_EN          = 0x1C,
       REG_WAKE_STAT        = 0x1D,
       REG_ANALOG_CTL       = 0x20,
       REG_CHANNEL          = 0x21,
       REG_RSSI             = 0x22,
       REG_PA               = 0x23,
       REG_CRYSTAL_ADJ      = 0x24,
       REG_VCO_CAL          = 0x26,
       REG_PWR_CTL          = 0x2E,
       REG_CARRIER_DETECT   = 0x2F,
       REG_CLOCK_MANUAL     = 0x32,
       REG_CLOCK_ENABLE     = 0x33,
       REG_SYN_LOCK_CNT     = 0x38,
       REG_MID1             = 0x3C,
       REG_MID2             = 0x3D,
       REG_MID3             = 0x3E,
       REG_MID4             = 0x3F
    };
    
    const uint8_t Read(const RADIO_REGISTERS address) const;
    void Write(const RADIO_REGISTERS address, const uint8_t value) const;
    
    const uint8_t pinReset_;    // active low reset
    const uint8_t pinChipSel_;  // chip select
};

#endif //  _CYWM6935_H_
