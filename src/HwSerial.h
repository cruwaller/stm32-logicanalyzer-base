#ifndef SERIAL_H_
#define SERIAL_H_

#include "hal/HardwareSerial.h"
#include "hal/platform.h"
#include "pinout.h"

class HwSerial : public HardwareSerial
{
public:
    HwSerial(uint32_t _rx, uint32_t _tx, int32_t duplex_pin = UNDEF_PIN, uint8_t inv = 0);

    void Begin(uint32_t baud, uint32_t config = SERIAL_8N1);
    void Pause(void);
    void Continue(void);

    void FAST_CODE_2 enable_receiver(void);
    void FAST_CODE_2 enable_transmitter(void);

    void FAST_CODE_2 flush_read()
    {
        while (available())
            (void)read();
    }

    size_t write(const uint8_t* buff, size_t len)
    {
        size_t ret;
        enable_transmitter();
        ret = HardwareSerial::write(buff, len);
        enable_receiver();
        return ret;
    }

private:
    struct gpio_out duplex_pin;
    uint8_t         duplex_pin_inv;
};

extern HwSerial Serial;

#endif /* SERIAL_H_ */
