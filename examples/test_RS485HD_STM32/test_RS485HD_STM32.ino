/**
 * test_RS485HD_STM32.ino - Test for Half-Duplex RS485 communication for Arduino_Core_STM32  
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
 *
 */
 
// Library
// --------------------
// PRDC RS485 Half Duplex
// Author: PRDC
#include <PRDC_RS485HD_STM32.h>

// Define variables
// --------------------
#define DE_PIN PB4 // RS485 transceiver DE pin
#define RE_PIN PB3 // RS485 transceiver RE pin

// Serial communications
#define SERIAL_BAUDRATE 1000000
PRDC_RS485HD_STM32 VFD_HS(USART1);

// LED control variables
unsigned long t_LED; // [us] time
unsigned long dt_LED = 1000; // [us] time difference
bool s_LED = true; // [-] LED state flag

// setup function
// --------------------
void setup() {
  // Define pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Communication ettings
  Serial.begin(SERIAL_BAUDRATE);
  VFD_HS.setPins(DE_PIN, RE_PIN);
  VFD_HS.begin(57600, SERIAL_7E1);
}

// loop function
// --------------------
void loop(){
  // Send command and read VFD
  Serial.println("~010A00000192\r");
  VFD_HS.print("~010A00000192\r");
  delay(10);
  while(VFD_HS.available()) {
    Serial.print((char)VFD_HS.read());
  }
  Serial.println();
}