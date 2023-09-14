// Test RS485HD STM32 
// Author: Milos Petrasinovic <mpetrasinovic@pr-dc.com>
// PR-DC, Republic of Serbia
// info@pr-dc.com
// --------------------

// Library
// --------------------
// PRDC RS485 Half Duplex
// Author: PRDC
#include <PRDC_RS485HD_STM32.h>

// Define variables
// --------------------
#define DE_PIN PB4 // MAX485 DE pin
#define RE_PIN PB3 // MAX485 RE pin

// Serial communications
#define protocolMax 65534
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
  VFD_HS.setPins(DE_PIN,RE_PIN);
  VFD_HS.begin(57600,SERIAL_7E1);
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