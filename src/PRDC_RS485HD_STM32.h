/**
 * PRDC_RS485HD_STM32.h - Half-Duplex RS485 communication for Arduino_Core_STM32  
 * Author: Milos Petrasinovic <mpetrasinovic@pr-dc.com>
 * PR-DC, Republic of Serbia
 * info@pr-dc.com
 * 
 * --------------------
 * Copyright (C) 2021 PR-DC <info@pr-dc.com>
 *
 * This file is part of PRDC_RS485HD_STM32.
 *
 * PRDC_RS485HD_STM32 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * PRDC_RS485HD_STM32 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with PRDC_RS485HD_STM32.  If not, see <https://www.gnu.org/licenses/>.
*/

/**
  This library is based on:
  
  HardwareSerial.h - Hardware serial library for Wiring
  Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
  
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
  
  Modified 28 September 2010 by Mark Sproul
  Modified 14 August 2012 by Alarus
  Modified 3 December 2013 by Matthijs Kooijman
*/

#ifndef PRDC_RS485HD_STM32_h
#define PRDC_RS485HD_STM32_h

#define RS485HD_STM32_AVAILABLE

#include <inttypes.h>
#include "Stream.h"

#include "uart.h"

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which head is the index of the location
// to which to write the next incoming character and tail is the index of the
// location from which to read.
// NOTE: a "power of 2" buffer size is reccomended to dramatically
//       optimize all the modulo operations for ring buffers.
// WARNING: When buffer sizes are increased to > 256, the buffer index
// variables are automatically increased in size, but the extra
// atomicity guards needed for that are not implemented. This will
// often work, but occasionally a race condition can occur that makes
// RS485 behave erratically. See https://github.com/arduino/Arduino/issues/2405
#if !defined(SERIAL_TX_BUFFER_SIZE)
#define SERIAL_TX_BUFFER_SIZE 64
#endif
#if !defined(SERIAL_RX_BUFFER_SIZE)
#define SERIAL_RX_BUFFER_SIZE 64
#endif
#if (SERIAL_TX_BUFFER_SIZE>256)
typedef uint16_t tx_buffer_index_t;
#else
typedef uint8_t tx_buffer_index_t;
#endif
#if (SERIAL_RX_BUFFER_SIZE>256)
typedef uint16_t rx_buffer_index_t;
#else
typedef uint8_t rx_buffer_index_t;
#endif

// A bool should be enough for this
// But it brings an build error due to ambiguous
// call of overloaded PRDC_RS485HD_STM32(int, int)
// So defining a dedicated type

// Define config for RS485.begin(baud, config);
// below configs are not supported by STM32
//#define RS485_5N1 0x00
//#define RS485_5N2 0x08
//#define RS485_5E1 0x20
//#define RS485_5E2 0x28
//#define RS485_5O1 0x30
//#define RS485_5O2 0x38
//#define RS485_6N1 0x02
//#define RS485_6N2 0x0A

#ifdef UART_WORDLENGTH_7B
#define RS485_7N1 0x04
#define RS485_7N2 0x0C
#define RS485_6E1 0x22
#define RS485_6E2 0x2A
#define RS485_6O1 0x32
#define RS485_6O2 0x3A
#endif
#define RS485_8N1 0x06
#define RS485_8N2 0x0E
#define RS485_7E1 0x24
#define RS485_8E1 0x26
#define RS485_7E2 0x2C
#define RS485_8E2 0x2E
#define RS485_7O1 0x34
#define RS485_8O1 0x36
#define RS485_7O2 0x3C
#define RS485_8O2 0x3E

class PRDC_RS485HD_STM32 : public Stream {
  protected:
    // Has any byte been written to the UART since begin()
    bool _written;

    // Don't put any members after these buffers, since only the first
    // 32 bytes of this struct can be accessed quickly using the ldd
    // instruction.
    unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];
    unsigned char _tx_buffer[SERIAL_TX_BUFFER_SIZE];

    serial_t _serial;

  public:
    explicit PRDC_RS485HD_STM32(uint32_t _rx, uint32_t _tx);
    explicit PRDC_RS485HD_STM32(PinName _rx, PinName _tx);
    explicit PRDC_RS485HD_STM32(void *peripheral);
    void begin(unsigned long baud)
    {
      begin(baud, RS485_8N1);
    }
    void begin(unsigned long, uint8_t);
    void setPins(uint32_t DE_PIN, uint32_t RE_PIN);
    void setPins(uint32_t E_PIN);
    void end();
    virtual int available(void);
    virtual int peek(void);
    virtual int read(void);
    int availableForWrite(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(long n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(unsigned int n)
    {
      return write((uint8_t)n);
    }
    inline size_t write(int n)
    {
      return write((uint8_t)n);
    }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool()
    {
      return true;
    }

    void setRx(uint32_t _rx);
    void setTx(uint32_t _tx);
    void setRx(PinName _rx);
    void setTx(PinName _tx);

    // Interrupt handlers
    static void _rx_complete_irq(serial_t *obj);
    static int _tx_complete_irq(serial_t *obj);
  private:
    uint32_t state;
    uint32_t pin_num;
    uint32_t enable_pin;
    uint32_t enable_pin2;
    uint8_t _config;
    unsigned long _baud;
    void init(PinName _rx, PinName _tx);
    void transmitEnable();
    static void transmitDisableIRQ(serial_t *obj);
    void transmitDisable();
};
#endif // PRDC_RS485HD_STM32_h
