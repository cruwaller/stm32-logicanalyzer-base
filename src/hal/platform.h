#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include "internal.h"
#include "irq.h"
#include <stddef.h>
#include <string.h>

#ifndef __section
#define __section(S) __attribute__((section(S)))
#endif
#define ATTR_NO_INIT __section(".noinit")

#define likely(x) __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)

#if RAM_CODE_IGNORE
#define FAST_CODE_1
#define FAST_CODE_2
#else  // !RAM_CODE_IGNORE
/* FAST_CODE_1 is always linked into RAM */
#define FAST_CODE_1 __section(".ram_code")
/* FAST_CODE_2 is linked into RAM only if enough space */
#if STM32F1xx || STM32L0xx || RAM_CODE_LIMITED
#define FAST_CODE_2
#else
#define FAST_CODE_2 __section(".ram_code")
#endif
#endif  // RAM_CODE_IGNORE

#define DRAM_ATTR
#define DMA_RAM_ATTR __section(".dma_data")
#define DRAM_FORCE_ATTR __section(".data")
#define DMA_ATTR WORD_ALIGNED_ATTR DMA_RAM_ATTR
#define WORD_ALIGNED_ATTR __attribute__((aligned(32)))

#define _DISABLE_IRQ() irq_disable()
#define _ENABLE_IRQ() irq_enable()
#define _SAVE_IRQ() irq_save()
#define _RESTORE_IRQ(_x) irq_restore(_x)

#define FORCED_INLINE inline __attribute__((always_inline))
#ifndef NO_INLINE
#define NO_INLINE __attribute__((noinline))
#endif
#define barrier() __asm__ __volatile__("" : : : "memory")

static inline void write_u32(void* addr, uint32_t val)
{
    barrier();
    *(volatile uint32_t*)addr = val;
}
static inline void write_u16(void* addr, uint16_t val)
{
    barrier();
    *(volatile uint16_t*)addr = val;
}
static inline void write_u8(void* addr, uint8_t val)
{
    barrier();
    *(volatile uint8_t*)addr = val;
}
static inline uint32_t read_u32(const void* addr)
{
    uint32_t val = *(volatile const uint32_t*)addr;
    barrier();
    return val;
}
static inline uint16_t read_u16(const void* addr)
{
    uint16_t val = *(volatile const uint16_t*)addr;
    barrier();
    return val;
}
static inline uint8_t read_u8(const void* addr)
{
    uint8_t val = *(volatile const uint8_t*)addr;
    barrier();
    return val;
}

// ----------------------------------

#define noInterrupts() __disable_irq()
#define interrupts() __enable_irq()

// ----------------------------------

extern void setup(void);
extern void loop(void);

// ----------------------------------

uint32_t millis(void);
uint32_t micros(void);

void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

// ----------------------------------

void platform_restart(void);
void platform_reboot_into_bootloader(const uint8_t* info);

// ----------------------------------

#endif /* _PLATFORM_H_ */
