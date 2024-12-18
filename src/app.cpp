#include "HwSerial.h"
#include "HwTimer.h"
#include "hal/debug.h"
#include <stdint.h>

#ifndef SERIAL_BAUDRATE
// #define SERIAL_BAUDRATE 1000000U
#define SERIAL_BAUDRATE 115200U
#endif

#if (GPIO_PIN_LED_RED != UNDEF_PIN)
struct gpio_out led_red;
#define LED_INIT_RED() led_red = gpio_out_setup(GPIO_PIN_LED_RED, GPIO_PIN_LED_RED_INV)
#define LED_STATE_RED(_x) gpio_out_write(led_red, ((_x) ^ GPIO_PIN_LED_RED_INV))
#else
#define LED_INIT_RED()
#define LED_STATE_RED(_x)
#endif

#if (GPIO_PIN_LED_GREEN != UNDEF_PIN)
struct gpio_out led_green;
#define LED_INIT_GREEN() led_green = gpio_out_setup(GPIO_PIN_LED_GREEN, GPIO_PIN_LED_GREEN_INV)
#define LED_STATE_GREEN(_x) gpio_out_write(led_green, ((_x) ^ GPIO_PIN_LED_GREEN_INV))
#else
#define LED_INIT_GREEN()
#define LED_STATE_GREEN(_x)
#endif

#if (GPIO_PIN_LED_BLUE != UNDEF_PIN)
struct gpio_out led_blue;
#define LED_INIT_BLUE() led_blue = gpio_out_setup(GPIO_PIN_LED_BLUE, GPIO_PIN_LED_BLUE_INV)
#define LED_STATE_BLUE(_x) gpio_out_write(led_blue, ((_x) ^ GPIO_PIN_LED_BLUE_INV))
#else
#define LED_INIT_BLUE()
#define LED_STATE_BLUE(_x)
#endif

#if (GPIO_PIN_LED_RED != UNDEF_PIN)
#define STATUS_LED(_x) LED_STATE_RED(_x)
#elif (GPIO_PIN_LED_GREEN != UNDEF_PIN)
#define STATUS_LED(_x) LED_STATE_GREEN(_x)
#endif

static struct gpio_in_group m_pins_in;

#if !defined(GPIO_PORT)
#error "GPIO config is invalid!"
#endif

#define INFO_ARRAY_SIZE 4096

typedef struct pin_info
{
    uint32_t us;
    uint32_t states;
} pin_info_t;

#define HEADER_BYTE '^'        // 0x5E //0xCA
#define TERMINATOR_BYTE1 0x0D  //'\r'  // 0x0D
#define TERMINATOR_BYTE2 0x0A  //'\n'  // 0x0A

enum commands {
    // Requests
    COMMAND_RUN  = 'r',
    COMMAND_HALT = 'h',
    COMMAND_GET  = 'g',
    // Responses
    COMMAND_ACK  = 'a',
    COMMAND_NACK = 'n',
    COMMAND_DATA = 'd',
};

typedef struct gpio_cfg
{
    uint8_t port;
    uint8_t pin;
} gpio_cfg_t;

typedef struct command_msg
{
    uint8_t header;
    uint8_t length;
    uint8_t command;
    uint8_t terminator1;
    uint8_t terminator2;
} command_msg_t;

typedef struct data_msg
{
    uint8_t header;
    uint8_t length;
    uint8_t command;
    // pad[1]
    uint32_t   count;  // 8B
    pin_info_t data[INFO_ARRAY_SIZE];
    uint8_t    align[2];  // 4B
    uint8_t    terminator1;
    uint8_t    terminator2;
} data_msg_t;

typedef union data_msg_buff {
    data_msg_t msg;
    uint8_t    buffer[sizeof(data_msg_t)];
} data_msg_buff_t;

static struct
{
    size_t next;
    union {
        uint8_t       buff[2 * sizeof(command_msg_t)];
        command_msg_t msg;
    };
} input_buffer;

/*********************************************/

constexpr uint16_t pin_mask = 0
#ifdef GPIO_PIN_APP
                            + (1 << GPIO_PIN_APP)
#endif
#ifdef GPIO_PIN_COEX0
                            + (1 << GPIO_PIN_COEX0)
#endif
#ifdef GPIO_PIN_COEX1
                            + (1 << GPIO_PIN_COEX1)
#endif
#ifdef GPIO_PIN_COEX2
                            + (1 << GPIO_PIN_COEX2)
#endif
    ;

static data_msg_buff_t m_data_resp;
static pin_info_t*     m_pin_info_ptr;
static size_t          m_pin_index;

static void FAST_CODE_1 pin_event_capture_cb(void)
{
    if (&m_data_resp.msg.data[INFO_ARRAY_SIZE] <= m_pin_info_ptr)
        return;
#if 0
    struct pin_info* const p_info = m_pin_info_ptr++;
    struct gpio_in_group   pins   = m_pins_in;

    p_info->us     = timer_counter_get();
    p_info->states = gpio_in_read_group(pins);
#elif 0
    //uint32_t const pr = EXTI->PR1;
    //EXTI->PR1 = pr;
    *m_pin_info_ptr = (struct pin_info){
        .us     = timer_counter_get(),
        .states = gpio_in_read_group(m_pins_in),
    };
    m_pin_info_ptr++;
#else
    struct pin_info* const p_info = m_pin_info_ptr++;
    *p_info = (struct pin_info){
        .us     = timer_counter_get(),
        .states = gpio_in_read_group(m_pins_in),
    };
#endif
}

/*********************************************/

void send_ack_nack(bool ok)
{
    static command_msg_t ack = {
        .header      = HEADER_BYTE,
        .length      = 3,
        .command = ok ? COMMAND_ACK : COMMAND_NACK,
        .terminator1 = TERMINATOR_BYTE1,
        .terminator2 = TERMINATOR_BYTE2,
    };
    Serial.write((uint8_t*)&ack, sizeof(ack));
}

void send_data_buff(void)
{
    size_t len = ((uintptr_t)m_pin_info_ptr - (uintptr_t)m_data_resp.msg.data);
    size_t count = len / sizeof(pin_info_t);
    if (count) {
        len += offsetof(data_msg_buff_t, msg.data);
        m_data_resp.msg.count     = count;
        m_data_resp.buffer[len++] = TERMINATOR_BYTE1;
        m_data_resp.buffer[len++] = TERMINATOR_BYTE2;
        Serial.write(m_data_resp.buffer, len);
    }
    send_ack_nack(true);
}

void processPacket(uint8_t command)
{
    switch (command) {
    case COMMAND_RUN: {
        LED_STATE_BLUE(true);
        // Clear data
        memset(&m_data_resp, 0, sizeof(m_data_resp));

        m_data_resp.msg.header  = HEADER_BYTE;
        m_data_resp.msg.command = COMMAND_DATA;

        m_pin_info_ptr = m_data_resp.msg.data;
        m_pin_index    = 0;
        // Start timer to track timing
        timer_counter_set(0);
        timer_enable();
        // Capture the initial state before enabling the ISRs
        pin_event_capture_cb();
        // Configure GPIO ISR
        gpio_in_group_isr(m_pins_in, pin_event_capture_cb, CHANGE);
        send_ack_nack(true);
        break;
    }
    case COMMAND_HALT: {
        gpio_in_group_isr_remove(m_pins_in);
        timer_disable();
        send_data_buff();
        LED_STATE_BLUE(false);
        break;
    }
    case COMMAND_GET: {
        send_data_buff();
        break;
    }
    default:
        platform_restart();
        break;
    }
}

void ParseInByte(uint8_t const inChar)
{
    uint8_t next = input_buffer.next;

    if (!next && HEADER_BYTE != inChar)
        return;

    input_buffer.buff[next++] = inChar;

    if (TERMINATOR_BYTE1 == input_buffer.msg.terminator1
        && TERMINATOR_BYTE2 == input_buffer.msg.terminator2) {
        if (input_buffer.msg.length == (next - 2)) {
            processPacket(input_buffer.msg.command);
        }
        input_buffer.msg.terminator1 = 0;
        input_buffer.msg.terminator2 = 0;
        next                         = 0;
    } else if (sizeof(input_buffer.buff) <= next) {
        next = 0;
    }
    input_buffer.next = next;
}

void setup()
{
    /*************** CONFIGURE LEDs *******************/
    LED_INIT_RED();
    LED_INIT_BLUE();
    LED_INIT_GREEN();
    LED_STATE_GREEN(1);

    delay(200);
#if defined(GPIO_PIN_DUMMY_IN_1)
    (void)gpio_in_setup(GPIO_PIN_DUMMY_IN_1, 0);
#endif
#if defined(GPIO_PIN_DUMMY_IN_2)
    (void)gpio_in_setup(GPIO_PIN_DUMMY_IN_2, 0);
#endif

    Serial.begin(SERIAL_BAUDRATE);
    // delay need to avoid weird input from ST-link
    delay(100);
    Serial.flush_read();
    // DEBUG_PRINTF("APP started\r\n");

    // m_pins_in = gpio_in_setup_group(GPIO_PORT, pin_mask, -1);
    m_pins_in = gpio_in_setup_group(GPIO_PORT, pin_mask, 0);

    timer_init();
}

void loop()
{
    uint32_t const  now = millis();
    static uint32_t now_last;

    if (1000 <= (now - now_last)) {
        static bool led_last = 0;
        STATUS_LED(led_last);
        led_last ^= true;
        now_last = now;
    }

    int available = Serial.available();
    while (available--) {
        ParseInByte(Serial.read());
    }
}
