/******************************************************************************

  BonoGPS: connect a GPS to mobile Apps to track lap times
  More info at https://github.com/renatobo/bonogps
  Renato Bonomini https://github.com/renatobo

  This file contains pinout definitions for a few tested boards:
  - https://docs.platformio.org/en/latest/boards/espressif32/esp32doit-devkit-v1.html
  - https://docs.platformio.org/en/latest/boards/espressif32/lolin_d32_pro.html

  If you have an undefined board, you need to
  - identify the LED_BUILTIN pin: this is usually the builtin blue led, used to signal WiFi status
  - identify the WIFI_MODE_BUTTON pin: this is BOOT for the DOIT-DevKit as it's already there, undefined for the LOLIN
  - identify UART2 PINs RX2 TX2, if not already defined by your board

******************************************************************************/

#define RX2 1 // Standard label Rx2 on board
#define TX2 3 // Standard label Tx2 on board
#define WIFI_MODE_BUTTON 0 // default is: use the boot button to switch wifi modes
#define LED_BUILTIN 2 // this should not be needed if you choose the right board