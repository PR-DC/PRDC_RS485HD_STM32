# PR-DC Half-Duplex RS485 communication Library for Arduino_Core_STM32

Library for Arduino_Core_STM32 and RS485 Half-Duplex transceivers. This library precisely changes the state of control pins (Receiver Output Enable and Driver Output Enable) for receiving and transmitting.

## Requirements
[Arduino](https://www.arduino.cc/)<br>
[Arduino_Core_STM32](https://github.com/stm32duino/Arduino_Core_STM32)<br>

This library is tested with 
**Arduino IDE 1.8.16** and **Arduino_Core_STM32 2.6.0**.

## Installation
Clone the repository to the Arduino libraries folder.

For example, on the Windows OS path is:
```
C:\Users\{User}\Documents\Arduino\libraries
```

For more information please visit: https://www.arduino.cc/en/guide/libraries#toc5

## Usage

For usage please check provided examples. 

On STM32F1, circular RX DMA is optional. Set `PRDC_RS485HD_RX_DMA=1` in the
global build flags for every translation unit, call `begin()` with 8N1, then
check the result of `beginReceiveDMA()`. USART1/2/3 use DMA1 channels 5/6/3;
the caller must reserve that channel. The existing receive ring is polled through
`available()`, `read()` and `peek()`; service it before one complete ring wraps.
TX and half-duplex direction control remain interrupt driven. DMA initialization
or transfer failure does not select IRQ reception as a fallback; retain a receive
timeout in the application. `end()` or a new `begin()` releases the DMA channel.
DMA is disabled by default, with no additional state on other targets.

## License
Copyright (C) 2023 PR-DC <info@pr-dc.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
