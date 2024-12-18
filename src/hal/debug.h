#pragma once

#include "HwSerial.h"
#include "printf.h"
#include "pinout.h"

#ifndef DEBUG_SERIAL
    //#define DEBUG_SERIAL Serial
#endif // ifndef DEBUG_SERIAL

#ifdef DEBUG_SERIAL
    /* Native "bare metal" implementation does not support print/println methods! */
    #define DEBUG_PRINTF(...)  Printf::printf_(__VA_ARGS__)
#else // !DEBUG_SERIAL
    // Debug disabled
    #define DEBUG_PRINTF(...)
#endif // DEBUG_SERIAL
