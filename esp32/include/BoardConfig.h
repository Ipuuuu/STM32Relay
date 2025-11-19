#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define TARGET_BOARD_STM32_BLUEPILL
// #define TARGET_BOARD_ARDUINO_UNO
// #define TARGET_BOARD_ARDUINO_NANO

// include board-specific pin maps
#if defined(TARGET_BOARD_STM32_BLUEPILL)
    #include "boards/STM32_Bluepill.h"

#elif defined(TARGET_BOARD_ARDUINO_UNO)
    #include "boards/Arduino_Uno.h"

#elif defined(TARGET_BOARD_ARDUINO_NANO)
    #include "boards/Arduino_Nano.h"

#else
    #error "No target board defined! Please define TARGET_BOARD_XXX"
#endif

#endif
