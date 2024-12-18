#pragma once

#include "hal/platform.h"
#include <stdint.h>

#define TimerIntervalUSDefault 20000

#define TIMER_SOON 40  // 40us

//#define TIMER_INLINE 1
#if TIMER_INLINE
#define TIMx TIM2
#define TIMx_IRQn TIM2_IRQn
#define TIMx_IRQx_FUNC TIM2_IRQHandler

static FORCED_INLINE uint32_t timer_counter_get(void)
{
    return TIMx->CNT;
}

static FORCED_INLINE void timer_counter_set(uint32_t const cnt)
{
    TIMx->CNT = cnt;
}

static FORCED_INLINE void timer_set(uint32_t const next)
{
    TIMx->ARR = next;
}
#endif

class HwTimer
{
public:
    typedef void (*timer_cb_t)(uint32_t us);

    HwTimer();
    void init();
    void start();
    void reset(int32_t offset = 0);
    void pause();
    void stop();

    inline void FAST_CODE_1 updateInterval(uint32_t const newTimerInterval)
    {
        HWtimerInterval = newTimerInterval;
    }
    inline bool FAST_CODE_1 isRunning(void) const
    {
        return running;
    }

    void callback();

    volatile timer_cb_t callbackTickPre;
    volatile timer_cb_t callbackTick;

    void setTime(uint32_t time = 0);

    void triggerSoon(void);

private:
    uint32_t HWtimerInterval;
    bool     running;
};

extern HwTimer timer1;

/* Low layer API - No IQR support when this is used! */
void     timer_init(void);
void     timer_enable(void);
void     timer_disable(void);
#if !TIMER_INLINE
uint32_t timer_counter_get(void);
void     timer_counter_set(uint32_t const cnt);
void     timer_set(uint32_t const next);
#endif
