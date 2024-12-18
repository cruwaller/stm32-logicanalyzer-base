#include "HwTimer.h"
#include "hal/irq.h"
#include "hal/priorities.h"

#if defined(STM32L4xx)
#define SWIER SWIER1
#endif

HwTimer timer1;

/****************************************************************
 * Low level timer code
 ****************************************************************/
#if !TIMER_INLINE
#define TIMx TIM2
#define TIMx_IRQn TIM2_IRQn
#define TIMx_IRQx_FUNC TIM2_IRQHandler

uint32_t FAST_CODE_1 timer_counter_get(void)
{
    return TIMx->CNT;
}

void FAST_CODE_1 timer_counter_set(uint32_t const cnt)
{
    TIMx->CNT = cnt;
}

void FAST_CODE_1 timer_set(uint32_t const next)
{
    TIMx->ARR = next;
}
#endif

/****************************************************************
 * HW Timer setup and irqs
 ****************************************************************/

extern "C"
{
    // Hardware timer IRQ handler - dispatch software timers
    void FAST_CODE_1 TIMx_IRQx_FUNC(void)
    {
        uint16_t const SR = TIMx->SR;
        if (SR & TIM_SR_UIF) {
            timer1.callback();
            TIMx->SR = SR & ~(TIM_SR_UIF);
        }
    }
}

void timer_enable(void)
{
    TIMx->DIER = TIM_DIER_UIE;
    TIMx->CR1 = TIM_CR1_CEN | TIM_CR1_URS /*| TIM_CR1_DIR*/;
    TIMx->SR &= ~(TIM_SR_UIF);
}

void timer_disable(void)
{
    TIMx->CR1 = 0;
    TIMx->DIER = 0;
}

static void timer_interrupt_enable(void)
{
    NVIC_SetPriority(TIMx_IRQn,
        NVIC_EncodePriority(NVIC_GetPriorityGrouping(), ISR_PRIO_TIM, 0));
    NVIC_EnableIRQ(TIMx_IRQn);
}

void timer_init(void)
{
    enable_pclock((uint32_t)TIMx);
    timer_disable();
    // Set clock prescaler to 1us
    // Note: PSC == 1 clock is APB1 x1 (36MHz) else x2 (72MHz)
    TIMx->PSC = (2 * get_pclock_frequency((uint32_t)TIMx) / 1000000) - 1;
    /*TIMx->ARR = (1U << 16) - 1;*/ // Init to max
    TIMx->CNT = 0;
    TIMx->EGR = TIM_EGR_UG;
}

/****************************************************************
 * Public
 ****************************************************************/
static void FAST_CODE_1 nullCallback(uint32_t) {};

HwTimer::HwTimer()
{
    callbackTickPre = nullCallback;
    callbackTick = nullCallback;

    HWtimerInterval = TimerIntervalUSDefault;
    running = false;
}

void FAST_CODE_1 HwTimer::callback()
{
    uint32_t const us = micros();
    callbackTickPre(us);
    callbackTick(us);
}

void HwTimer::init()
{
    timer_init();
    timer_interrupt_enable();
}

void HwTimer::start()
{
    reset(0);
    timer_enable();
}

void HwTimer::stop()
{
    timer_disable();
}

void HwTimer::pause()
{
    stop();
}

void FAST_CODE_1 HwTimer::reset(int32_t const offset)
{
    /* Reset counter and set next alarm time */
    timer_set(HWtimerInterval - offset);
    timer_counter_set(HWtimerInterval - offset);
}

void FAST_CODE_1 HwTimer::setTime(uint32_t time)
{
    if (!time)
        time = HWtimerInterval;
    timer_set(time);
    timer_counter_set(time);
}

void FAST_CODE_1 HwTimer::triggerSoon(void)
{
#if 1
    timer_counter_set(TIMER_SOON);
#else
    /* Generate soft trigger to run ISR asap */
    EXTI->SWIER |= (0x1 << 3);
#endif
}
