#pragma once

#include "hal/platform.h"

#define EMPTY()
#define UNDEF_PIN (-1)

/**********************************
           DEFAULTS
 **********************************/

#ifndef GPIO_PIN_UART_RX
#define GPIO_PIN_UART_RX UNDEF_PIN
#endif
#ifndef GPIO_PIN_UART_TX
#define GPIO_PIN_UART_TX UNDEF_PIN
#endif

#ifndef UART_USE_DMA
#define UART_USE_DMA 1
#endif
#ifndef UART_INVERTED
#define UART_INVERTED 0
#endif

#ifndef BUFFER_OE
#define BUFFER_OE UNDEF_PIN
#endif
#ifndef BUFFER_OE_INVERTED
#define BUFFER_OE_INVERTED 0
#endif

#ifndef GPIO_PIN_LED_RED
#define GPIO_PIN_LED_RED UNDEF_PIN
#endif
#ifndef GPIO_PIN_LED_RED_INV
#define GPIO_PIN_LED_RED_INV 0
#endif
#ifndef GPIO_PIN_LED_GREEN
#define GPIO_PIN_LED_GREEN UNDEF_PIN
#endif
#ifndef GPIO_PIN_LED_GREEN_INV
#define GPIO_PIN_LED_GREEN_INV 0
#endif
#ifndef GPIO_PIN_LED_BLUE
#define GPIO_PIN_LED_BLUE UNDEF_PIN
#endif
#ifndef GPIO_PIN_LED_BLUE_INV
#define GPIO_PIN_LED_BLUE_INV 0
#endif

#ifndef GPIO_PIN_BUTTON
#define GPIO_PIN_BUTTON UNDEF_PIN
#endif

#ifndef GPIO_PIN_DEBUG_RX
#define GPIO_PIN_DEBUG_RX UNDEF_PIN
#endif
#ifndef GPIO_PIN_DEBUG_TX
#define GPIO_PIN_DEBUG_TX UNDEF_PIN
#endif
