
### WS2812B on STM32F0303F4-DEV V1.0 board

#### Hardware requirements
Based on STM32F030F4-DEV V1.0 board @ brobwind.com

| Device  | configuration            |
| ------- | ------------------------ |
| MCU     | STM32F030F4P6            |
| HSE     | 8MHz (Optional)          |
| USART1  | TX - PA2, RX - PA3       |
| WS2812B | DIN - PA9                |

#### WS2812B datasheet
https://acrobotic.com/datasheets/WS2812B.pdf

#### ChibiOS/RT based RTOS
Based on Commit ID: 31ab758

Board information:
```
Kernel:       4.0.0
Compiler:     GCC 4.9.3 20150303 (release) [ARM/embedded-4_9-branch revision 221220]
Architecture: ARMv6-M
Core Variant: Cortex-M0
Port Info:    Preemption through NMI
Platform:     STM32F030x6 Entry Level Value Line devices
Board:        STM32F030F4-DEV V1.0
Build time:   Jan 30 2017 - 19:21:56
```

#### Usage
Run command on serial port USART1 @ 115200,8n1

Shell command led will update the WS2812B, syntax:
```
led <r> <g> <b>
```
Note: r, g and b using hex format

```
ChibiOS/RT Shell
ch> led 00 ff 00
led: r=0, g=255, b=0
ch> led 00 3 00
led: r=0, g=3, b=0
```

#### For detail info, please refer:
https://www.brobwind.com/archives/1027
