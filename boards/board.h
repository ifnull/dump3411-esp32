/*
 * board.h - the one header the firmware includes for board information.
 *
 * Pulls in the selected boards/board_<name>.h (see boards/CMakeLists.txt)
 * and checks it against the Kconfig board entry in both directions: an
 * enabled peripheral must have its pins, and a header must not define pins
 * for something its Kconfig entry doesn't declare. Board headers hold
 * BOARD_NAME and pin macros only; pins for absent peripherals stay
 * undefined, so using one fails to compile.
 */
#ifndef BOARD_H
#define BOARD_H

#include "sdkconfig.h"
#include DUMP3411_BOARD_HEADER

#ifndef BOARD_NAME
#error "board header must define BOARD_NAME"
#endif

#if defined(CONFIG_DUMP3411_ENABLE_EINK_DISPLAY) &&                           \
    !(defined(BOARD_EPD_PIN_RST) && defined(BOARD_EPD_PIN_CS) &&               \
      defined(BOARD_EPD_PIN_BUSY) && defined(BOARD_EPD_PIN_DC) &&              \
      defined(BOARD_EPD_PIN_SCK) && defined(BOARD_EPD_PIN_MOSI))
#error "e-ink display enabled, but the board header lacks BOARD_EPD_PIN_*"
#endif
#if defined(BOARD_EPD_PIN_CS) && !defined(CONFIG_DUMP3411_BOARD_HAS_EINK)
#error "board header has e-ink pins, but its Kconfig entry doesn't select DUMP3411_BOARD_HAS_EINK"
#endif

#if defined(CONFIG_DUMP3411_ENABLE_BUTTONS) && !defined(BOARD_BUTTON_COUNT)
#error "buttons enabled, but the board header lacks BOARD_BUTTON_*"
#endif
#if defined(BOARD_BUTTON_COUNT) && !defined(CONFIG_DUMP3411_BOARD_HAS_BUTTONS)
#error "board header has button pins, but its Kconfig entry doesn't select DUMP3411_BOARD_HAS_BUTTONS"
#endif

#if defined(CONFIG_DUMP3411_ENABLE_GNSS) &&                                   \
    !(defined(BOARD_GNSS_UART_TX_PIN) && defined(BOARD_GNSS_UART_RX_PIN))
#error "GNSS enabled, but the board header lacks BOARD_GNSS_UART_*"
#endif
#if defined(BOARD_GNSS_UART_TX_PIN) && !defined(CONFIG_DUMP3411_BOARD_HAS_GNSS)
#error "board header has GNSS pins, but its Kconfig entry doesn't select DUMP3411_BOARD_HAS_GNSS"
#endif

#endif /* BOARD_H */
