#include "platform.h"
#include "debug.h"
#include "gpio.h"
#include "pinout.h"
#include "stm32_eeprom.h"

enum {
    BL_FLAG_KEY = 0x626C0000,
    /* 16bits */
    BL_FLAG_BAUDRATE = 1,
};

struct bootloader
{
    uint32_t key;
    uint32_t reset_type;
    uint32_t flags;
    uint32_t baudrate;
};

void platform_restart(void)
{
    NVIC_SystemReset();
}

void platform_reboot_into_bootloader(const uint8_t* info)
{
#if 0
    if (validate_bl_indentifier(info) < 0)
        return;
    /* Fill reset info into RAM for bootloader */
    extern __IO uint32_t _bootloader_data;
    volatile struct bootloader * blinfo = ((struct bootloader*)&_bootloader_data) + 0;
    blinfo->key = 0x454c5253; // ELRS
    blinfo->reset_type = 0xACDC;
    blinfo->flags = BL_FLAG_KEY | BL_FLAG_BAUDRATE;
    blinfo->baudrate = RX_BAUDRATE;
#endif
    platform_restart();
}
