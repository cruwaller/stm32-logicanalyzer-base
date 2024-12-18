#pragma once

// USART3 is connected to ST-Link VCOM
#define GPIO_PIN_UART_RX GPIO('D', 9)
#define GPIO_PIN_UART_TX GPIO('D', 8)

#define GPIO_PIN_LED_GREEN GPIO('B', 0)
#define GPIO_PIN_LED_BLUE GPIO('B', 7)
#define GPIO_PIN_LED_RED GPIO('B', 14)

// User button on PC13
#define GPIO_PORT 'C'
#define GPIO_PIN_APP 15
#define GPIO_PIN_COEX0 14
#define GPIO_PIN_COEX2 13
#define GPIO_PIN_COEX1 12
