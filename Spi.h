/*
  Spi.h - SPI library
  Copyright (c) 2008 Cam Thompson.
  Author: Cam Thompson, Micromega Corporation, <www.micromegacorp.com>
  Version: December 15, 2008

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef Spi_h
#define Spi_h

#include "WProgram.h"

class SPI
{
public:
    SPI(void);
    void init();
    void mode(byte);
    byte transfer(byte);
    byte transfer(byte, byte);
    
private:
    static const uint8_t pinSCK_  = 13;
    static const uint8_t pinMISO_ = 12;
    static const uint8_t pinMOSI_ = 11;
    static const uint8_t pinSS_   = 10;
};

extern SPI Spi;

#endif
