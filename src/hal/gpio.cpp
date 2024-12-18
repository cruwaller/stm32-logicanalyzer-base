// Code to setup clocks and gpio on stm32f1
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
// https://github.com/KevinOConnor/klipper
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "gpio.h"  // gpio_out_setup
#include "helpers.h"
#include "internal.h"
#include "irq.h"
#include "platform.h"
#include "priorities.h"
#include "stm32_def.h"
#include <string.h>  // ffs

#if defined(STM32L4xx) || defined(STM32G0xx)
#define IMR IMR1
#define EMR EMR1
#define RTSR RTSR1
#define FTSR FTSR1
#if defined(STM32G0xx)
/* TODO: Should merge these 2 */
// #define PR FPR1
#define PR RPR1
#else
#define PR PR1
#endif
#endif

GPIO_TypeDef* __section(".data") digital_regs[] = {
    ['A' - 'A'] = GPIOA, GPIOB, GPIOC,
#ifdef GPIOD
    ['D' - 'A'] = GPIOD,
#else
    NULL,
#endif
#ifdef GPIOE
    ['E' - 'A'] = GPIOE,
#else
    NULL,
#endif
#ifdef GPIOF
    ['F' - 'A'] = GPIOF,
#else
    NULL,
#endif
#ifdef GPIOG
    ['G' - 'A'] = GPIOG,
#else
    NULL,
#endif
#ifdef GPIOH
    ['H' - 'A'] = GPIOH,
#else
    NULL,
#endif
#ifdef GPIOI
    ['I' - 'A'] = GPIOI,
#else
    NULL,
#endif
};

// Convert a register pointer to index
static int regs_to_port_idx(GPIO_TypeDef* regs)
{
    uint32_t i;
    for (i = 0; i < ARRAY_SIZE(digital_regs); i++)
        if (digital_regs[i] == regs)
            return i;
    return -1;
}

// Convert a register and bit location back to an integer pin identifier
static int regs_to_pin(GPIO_TypeDef* regs, uint32_t bit)
{
    uint32_t i = regs_to_port_idx(regs);
    if (i < 0)
        return 0;
    return GPIO('A' + i, ffs(bit) - 1);
}

struct gpio_out gpio_out_setup(uint32_t pin, uint32_t val)
{
    if (GPIO('J', 0) <= pin)
        return {.regs = NULL, .bit = 0};
    GPIO_TypeDef* regs = digital_regs[GPIO2PORT(pin)];
    if (!regs)
        Error_Handler();
    struct gpio_out g = {.regs = regs, .bit = GPIO2BIT(pin)};
    gpio_out_reset(g, val);
    return g;
}

void gpio_out_reset(struct gpio_out g, uint32_t val)
{
    GPIO_TypeDef* regs = (GPIO_TypeDef*)g.regs;
    int           pin  = regs_to_pin(regs, g.bit);
    irqstatus_t   flag = irq_save();
    gpio_peripheral(pin, GPIO_OUTPUT, 0);
    gpio_out_write(g, val);
    irq_restore(flag);
}

void FAST_CODE_1 gpio_out_toggle_noirq(struct gpio_out g)
{
    GPIO_TypeDef* regs = (GPIO_TypeDef*)g.regs;
    regs->ODR ^= g.bit;
}

void FAST_CODE_1 gpio_out_toggle(struct gpio_out g)
{
    irqstatus_t flag = irq_save();
    gpio_out_toggle_noirq(g);
    irq_restore(flag);
}

void FAST_CODE_1 gpio_out_write(struct gpio_out g, uint32_t val)
{
    GPIO_TypeDef* regs = (GPIO_TypeDef*)g.regs;
    if (val)
        regs->BSRR = g.bit;
    else
        regs->BSRR = g.bit << GPIO_NUM_PINS;
}

uint8_t FAST_CODE_1 gpio_out_read(struct gpio_out g)
{
    GPIO_TypeDef* regs = (GPIO_TypeDef*)g.regs;
    return !!(regs->ODR & g.bit);
}

struct gpio_in_group gpio_in_setup_group(uint8_t port, uint16_t pins, int32_t pull_up)
{
    port -= 'A';

    if (ARRAY_SIZE(digital_regs) <= port)
        return {.regs = NULL, .bits = 0};

    GPIO_TypeDef* regs = digital_regs[port];
    if (!regs)
        Error_Handler();
    struct gpio_in_group g = {.regs = regs, .bits = pins};
    gpio_in_reset_group(g, pull_up);
    return g;
}

void gpio_in_reset_group(struct gpio_in_group g, int32_t pull_up)
{
    uint32_t const    port_idx = regs_to_port_idx((GPIO_TypeDef*)g.regs);
    irqstatus_t const flag     = irq_save();
    for (size_t iter = 0; iter < GPIO_NUM_PINS; iter++)
        if (g.bits & (1 << iter))
            gpio_peripheral(GPIO('A' + port_idx, iter), GPIO_INPUT, pull_up);
    irq_restore(flag);
}

uint16_t FAST_CODE_1 gpio_in_read_group(struct gpio_in_group g)
{
    //GPIO_TypeDef* regs = (GPIO_TypeDef*)g.regs;
    //return (regs->IDR & g.bits);
    return ((GPIO_TypeDef*)g.regs)->IDR;
}

struct gpio_in gpio_in_setup(uint32_t pin, int32_t pull_up)
{
    if (GPIO('J', 0) <= pin)
        return {.regs = NULL, .bit = 0};

    GPIO_TypeDef* regs = digital_regs[GPIO2PORT(pin)];
    if (!regs)
        Error_Handler();
    struct gpio_in g = {.regs = regs, .bit = GPIO2BIT(pin)};
    gpio_in_reset(g, pull_up);
    return g;
}

void gpio_in_reset(struct gpio_in g, int32_t pull_up)
{
    GPIO_TypeDef* regs = (GPIO_TypeDef*)g.regs;
    int           pin  = regs_to_pin(regs, g.bit);
    irqstatus_t   flag = irq_save();
    gpio_peripheral(pin, GPIO_INPUT, pull_up);
    irq_restore(flag);
}

uint8_t FAST_CODE_1 gpio_in_read(struct gpio_in g)
{
    GPIO_TypeDef* regs = (GPIO_TypeDef*)g.regs;
    return !!(regs->IDR & g.bit);
}

typedef struct
{
    isr_cb_t callback;
} gpio_irq_conf_str;

/* Private Variables */
static gpio_irq_conf_str DRAM_FORCE_ATTR gpio_irq_conf[GPIO_NUM_PINS];

#if defined(STM32F0xx) || defined(STM32G0xx) || defined(STM32L0xx)
static void FAST_CODE_1 gpio_isr_0_1_cb(void)
{
    uint32_t const irqs = EXTI->PR;
    uint8_t        pin;
    for (pin = 0; pin <= 1; pin++) {
        if (irqs & (1 << pin)) {
            if (gpio_irq_conf[pin].callback != NULL)
                gpio_irq_conf[pin].callback();
        }
    }
}

static void FAST_CODE_1 gpio_isr_2_3_cb(void)
{
    uint32_t const irqs = EXTI->PR;
    uint8_t        pin;
    for (pin = 2; pin <= 3; pin++) {
        if (irqs & (1 << pin)) {
            if (gpio_irq_conf[pin].callback != NULL)
                gpio_irq_conf[pin].callback();
        }
    }
}

static void FAST_CODE_1 gpio_isr_4_15_cb(void)
{
    uint32_t const irqs = EXTI->PR;
    uint8_t        pin;
    for (pin = 4; pin <= 15; pin++) {
        if (irqs & (1 << pin)) {
            if (gpio_irq_conf[pin].callback != NULL)
                gpio_irq_conf[pin].callback();
        }
    }
}
#else
static void FAST_CODE_1 gpio_isr_15_10_cb(void)
{
    uint32_t const irqs = EXTI->PR;
    uint8_t        pin;
    for (pin = 10; pin <= 15; pin++) {
        if (irqs & (1 << pin)) {
            if (gpio_irq_conf[pin].callback != NULL)
                gpio_irq_conf[pin].callback();
        }
    }
    EXTI->PR = irqs;
}

static void FAST_CODE_1 gpio_isr_9_5_cb(void)
{
    uint32_t const irqs = EXTI->PR;
    uint8_t        pin;
    for (pin = 5; pin <= 9; pin++) {
        if (irqs & (1 << pin)) {
            if (gpio_irq_conf[pin].callback != NULL)
                gpio_irq_conf[pin].callback();
        }
    }
    EXTI->PR = irqs;
}
#endif

static void in_isr_configure(
    uint32_t const port_idx, uint32_t const pin_idx, uint8_t const it_mode, IRQn_Type irqnb)
{
    uint32_t const pin_bit = 1 << pin_idx;

    /* Configure the External Interrupt or event for the current IO */
#ifdef AFIO_BASE
    __IO uint32_t* exticr_reg = &AFIO->EXTICR[pin_idx >> 2u];
#elif defined(EXTI) && defined(STM32G0xx)
#define EXTI_IDX_OFFSET 0x8
    __IO uint32_t* exticr_reg = &EXTI->EXTICR[pin_idx >> 2u];
#elif defined(SYSCFG_BASE)
    __IO uint32_t* exticr_reg = &SYSCFG->EXTICR[pin_idx >> 2u];
#endif
#ifndef EXTI_IDX_OFFSET
#define EXTI_IDX_OFFSET 0x4
#endif
    uint32_t EXTICR = *exticr_reg;
    CLEAR_BIT(EXTICR, (0x0FU << (EXTI_IDX_OFFSET * (pin_idx & 0x03))));
    SET_BIT(EXTICR, (port_idx << (EXTI_IDX_OFFSET * (pin_idx & 0x03))));
    *exticr_reg = EXTICR;

    /* Configure the interrupt mask */
    if ((it_mode & GPIO_MODE_IT) == GPIO_MODE_IT) {
        SET_BIT(EXTI->IMR, pin_bit);
    } else {
        CLEAR_BIT(EXTI->IMR, pin_bit);
    }

    /* Configure the event mask */
    if ((it_mode & GPIO_MODE_EVT) == GPIO_MODE_EVT) {
        SET_BIT(EXTI->EMR, pin_bit);
    } else {
        CLEAR_BIT(EXTI->EMR, pin_bit);
    }

    /* Enable or disable the rising trigger */
    if ((it_mode & RISING) == RISING) {
        SET_BIT(EXTI->RTSR, pin_bit);
    } else {
        CLEAR_BIT(EXTI->RTSR, pin_bit);
    }

    /* Enable or disable the falling trigger */
    if ((it_mode & FALLING) == FALLING) {
        SET_BIT(EXTI->FTSR, pin_bit);
    } else {
        CLEAR_BIT(EXTI->FTSR, pin_bit);
    }
    // Clear pending IRQ
    EXTI->PR = pin_bit;
    // Enable and set EXTI Interrupt
    NVIC_SetPriority(irqnb, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_EXTI, 0));
    NVIC_EnableIRQ(irqnb);
}

void gpio_in_isr(struct gpio_in g, isr_cb_t callback, uint8_t it_mode)
{
    GPIO_TypeDef*  regs = (GPIO_TypeDef*)g.regs;
    uint32_t const pin  = regs_to_pin(regs, g.bit);
    if (!regs)
        Error_Handler();
    uint32_t const port_idx = GPIO2PORT(pin);
    uint32_t const index    = GPIO2IDX(pin);

    IRQn_Type irq = (IRQn_Type)gpio_irq_get(index);

    in_isr_configure(port_idx, index, it_mode, irq);

#if defined(STM32F0xx) || defined(STM32G0xx) || defined(STM32L0xx)
    if (3 < index) {
        gpio_isr_set(gpio_isr_4_15_cb, irq);
    } else if (1 < index) {
        gpio_isr_set(gpio_isr_2_3_cb, irq);
    } else {
        gpio_isr_set(gpio_isr_0_1_cb, irq);
    }
    write_u32(&gpio_irq_conf[index].callback, (uint32_t)callback);
#else
    if (9 < index) {
        gpio_isr_set(gpio_isr_15_10_cb, irq);
        write_u32(&gpio_irq_conf[index].callback, (uint32_t)callback);
    } else if (4 < index) {
        gpio_isr_set(gpio_isr_9_5_cb, irq);
        write_u32(&gpio_irq_conf[index].callback, (uint32_t)callback);
    } else {
        gpio_isr_set(callback, irq);
    }
#endif
}

void gpio_in_isr_remove(struct gpio_in const g)
{
    if (!gpio_in_valid(g))
        Error_Handler();
    uint32_t    index = ffs(g.bit) - 1;
    irqstatus_t irq   = irq_save();

#if defined(STM32F0xx) || defined(STM32G0xx) || defined(STM32L0xx)
    write_u32(&gpio_irq_conf[index].callback, 0);
#else
    if (9 < index) {
        write_u32(&gpio_irq_conf[index].callback, 0);
    } else if (4 < index) {
        write_u32(&gpio_irq_conf[index].callback, 0);
    } else {
        gpio_isr_set(0, (IRQn_Type)gpio_irq_get(index));
    }
#endif
    irq_restore(irq);
}

void FAST_CODE_1 gpio_in_isr_clear_pending(struct gpio_in const g)
{
    if (gpio_in_valid(g))
        // Clear pending IRQ
        EXTI->PR = g.bit;
}

#if defined(STM32F3xx)
#define EXTI2_IRQn EXTI2_TSC_IRQn
#endif

void gpio_in_group_isr(struct gpio_in_group g, isr_cb_t func, uint8_t type)
{
    if (!g.regs)
        Error_Handler();
    uint32_t const port_idx = regs_to_port_idx((GPIO_TypeDef*)g.regs);
    if (port_idx < 0)
        Error_Handler();
    for (size_t index = 0; index < GPIO_NUM_PINS; index++) {
        if (g.bits & (1 << index)) {
            in_isr_configure(port_idx, index, type, (IRQn_Type)gpio_irq_get(index));
        }
    }

    irqstatus_t flag = irq_save();
#if defined(STM32F0xx) || defined(STM32G0xx) || defined(STM32L0xx)
    if (g.bits & 0xfff0)
        gpio_isr_set(func, EXTI4_15_IRQn);
    if (g.bits & 0xc)
        gpio_isr_set(func, EXTI2_3_IRQn);
    if (g.bits & 0x3)
        gpio_isr_set(func, EXTI0_1_IRQn);
#else
    if (g.bits & 0xFE00)
        gpio_isr_set(func, EXTI15_10_IRQn);
    if (g.bits & 0x3E0)
        gpio_isr_set(func, EXTI9_5_IRQn);
    if (g.bits & (1 << 4))
        gpio_isr_set(func, EXTI4_IRQn);
    if (g.bits & (1 << 3))
        gpio_isr_set(func, EXTI3_IRQn);
    if (g.bits & (1 << 2))
        gpio_isr_set(func, EXTI2_IRQn);
    if (g.bits & (1 << 1))
        gpio_isr_set(func, EXTI1_IRQn);
    if (g.bits & (1 << 0))
        gpio_isr_set(func, EXTI0_IRQn);
#endif
    irq_restore(flag);
}

void gpio_in_group_isr_remove(struct gpio_in_group g)
{
    if (!g.bits)
        return;
    irqstatus_t flag = irq_save();
#if defined(STM32F0xx) || defined(STM32G0xx) || defined(STM32L0xx)
    if (g.bits & 0xfff0)
        gpio_isr_set(0, EXTI4_15_IRQn);
    if (g.bits & 0xc)
        gpio_isr_set(0, EXTI2_3_IRQn);
    if (g.bits & 0x3)
        gpio_isr_set(0, EXTI0_1_IRQn);
#else
    if (g.bits & 0xFE00)
        gpio_isr_set(0, EXTI15_10_IRQn);
    if (g.bits & 0x3E0)
        gpio_isr_set(0, EXTI9_5_IRQn);
    if (g.bits & (1 << 4))
        gpio_isr_set(0, EXTI4_IRQn);
    if (g.bits & (1 << 3))
        gpio_isr_set(0, EXTI3_IRQn);
    if (g.bits & (1 << 2))
        gpio_isr_set(0, EXTI2_IRQn);
    if (g.bits & (1 << 1))
        gpio_isr_set(0, EXTI1_IRQn);
    if (g.bits & (1 << 0))
        gpio_isr_set(0, EXTI0_IRQn);
#endif
    irq_restore(flag);
}

void FAST_CODE_1 gpio_in_group_isr_clear_pending(struct gpio_in_group g)
{
    if (!g.bits)
        return;
    EXTI->PR = g.bits;
}

/*********************/

struct gpio_adc gpio_adc_setup(uint32_t pin)
{
    return {.adc = NULL, .chan = 0};
}

uint32_t gpio_adc_read(struct gpio_adc g)
{
    return 0;
}
