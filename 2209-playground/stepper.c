#include <stdio.h>
#include <assert.h>
#include <pico/stdlib.h>

#include "stepper.h"
#include <steps.pio.h>
#include <pulse_length.pio.h>

#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/uart.h>

const int direction = 5;
const int step = 6;
const int enable = 7;
const int idx = 8;
const int diag = 9;
const int tmc2209uart = 10;

const int uart_rx = 21;
const int uart_tx = 20;

const int ms1 = 14;
const int ms2 = 15;

int micro_steps = 8;

const bool away = false;
const bool toward = true;

uint program_offset = 0;

uint global_baudrate = 100000;

const uint8_t preamble = 0x05;

uint8_t tmc2209crc_accumulate(uint8_t payload, uint8_t crc);

void init_uart_hw() {
    uart_init(uart1, global_baudrate);

    gpio_set_function(uart_tx, UART_FUNCSEL_NUM(uart1, uart_tx));
    gpio_set_function(uart_rx, UART_FUNCSEL_NUM(uart1, uart_rx));
}

void stepper_step_irq_handler() {
    printf("** stepper step irq\n");
    pio_interrupt_clear(pio0, 0);
}

void init_steps_pio() {
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    program_offset = pio_add_program(pio, &steps_program);
    pio_sm_config c = steps_program_get_default_config(program_offset);


    sm_config_set_set_pins(&c, step, 1);
    sm_config_set_sideset_pins(&c, 0);
    pio_sm_set_consecutive_pindirs(pio, sm, step, 1, true);
    sm_config_set_clkdiv_int_frac(&c, 5000, 0);
    sm_config_set_out_special(&c, true, false, false);

    irq_add_shared_handler(pio_get_irq_num(pio, 0), stepper_step_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    pio_set_irqn_source_enabled(pio, 0, pis_interrupt0, true);
    irq_set_enabled(pio_get_irq_num(pio, 0), true);

    pio_sm_init(pio, sm, program_offset, &c);
    pio_gpio_init(pio, step);
    pio_sm_set_enabled(pio, sm, true);
}

void init_pulse_length_pio() {
    PIO pio = pio0;
    uint sm = pio_claim_unused_sm(pio, true);
    program_offset = pio_add_program(pio, &pulse_length_program);
    pio_sm_config c = pulse_length_program_get_default_config(program_offset);
    sm_config_set_clkdiv_int_frac(&c, 5000, 0);
    pio_sm_init(pio, sm, program_offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void init_stepper() {
    gpio_init(direction);
    gpio_set_dir(direction, GPIO_OUT);
    gpio_put(direction, true);

//    gpio_init(step);
//    gpio_set_dir(step, GPIO_OUT);
//    gpio_put(step, false);

    gpio_init(idx);
    gpio_set_dir(idx, GPIO_IN);

    gpio_init(diag);
    gpio_set_dir(diag, GPIO_IN);

    gpio_init(enable);
    gpio_set_dir(enable, GPIO_OUT);
    gpio_put(enable, false);

    gpio_init(ms1);
    gpio_set_dir(ms1, GPIO_OUT);
    gpio_put(ms1, false);

    gpio_init(ms2);
    gpio_set_dir(ms2, GPIO_OUT);
    gpio_put(ms2, false);

    init_steps_pio();
    init_pulse_length_pio();

    init_uart_hw();
}

void enable_motor() {
    gpio_put(enable, false);
}

void disable_motor() {
    gpio_put(enable, true);
}

void step_motor() {
    gpio_put(step, false);
    sleep_us(40);
    gpio_put(step, true);
    sleep_us(40);
    if (gpio_get(diag)) {
        printf("help!\n");
    }
}

void step_pio(size_t steps) {
    pio_sm_put_blocking(pio0, 0, steps * micro_steps);
    while (!pio_interrupt_get(pio0, 0)) {
        tight_loop_contents();
    }
}

void rotate(bool dir) {
    gpio_put(direction, dir);
#if 0
    for (int i = 0; i < 200; i++) {
        for (int m = 0; m < micro_steps; m++) {
            step_motor();
        }
    }
#else
    step_pio(200);
#endif
}

void move(bool dir, int distance, int delay) {
    gpio_put(direction, dir);

    int micro_steps_taken = 0;
    int cycles = 0;

    step_pio(distance);

    micro_steps_taken += distance * micro_steps;
    int full_steps = micro_steps_taken / micro_steps;
    int rotations = full_steps / 200;

    printf("microsteps: %d\n", micro_steps_taken);
    printf("cycles: %d\n", cycles);
    printf("full steps: %d\n", full_steps);
    printf("rotations: %d\n", rotations);
}

void dance() {
    move(away, 1000, 10);
    move(toward, 1000, 1);
}

uint8_t tmc2209crc_accumulate(uint8_t payload, uint8_t crc) {
    for (size_t j = 0; j < 8; j++) {
        if ((crc >> 7) ^ (payload & 0x01)) {
            crc = (crc << 1) ^ 0x07;
        } else {
            crc = (crc << 1);
        }
        payload = payload >> 1;
    }
    return crc;
}


bool validate_reply(uint8_t* reply, uint8_t address_request) {

    uint8_t reply_crc = 0;
    for (int i = 0; i < 7; i++) {
        reply_crc = tmc2209crc_accumulate(reply[i], reply_crc);
    }
    if (reply[7] != reply_crc) {
        printf("invalid crc\n");
        return false;
    }
    if ((reply[0] & 0x0f) != preamble) {
        printf("invalid preamble\n");
        return false;
    }
    if (reply[1] != 0xFF) {
        printf("invalid master address\n");
        return false;
    }
    if ((reply[2] & 0x80) != 0x0) {
        printf("invalid request address\n");
        return false;
    }
    if ((reply[2] & 0x7f) != address_request) {
        printf("incorrect request address\n");
        return false;
    }
    return true;
}

void uart_tmc2209_put_byte(uint8_t byte, uint8_t* crc) {
    if (crc) {
        *crc = tmc2209crc_accumulate(byte, *crc);
    }
    uart_putc_raw(uart1, byte);
    uint8_t loop = uart_getc(uart1);
    assert(byte == loop);
}

uint8_t uart_tmc2209_get_byte() {
    return uart_getc(uart1);
}

bool read_register(uint8_t reg, uint32_t* value) {

    uint8_t wire_address = reg & 0x7;
    uint8_t message[4] = { preamble, 0x00, wire_address ,0 };

    for (size_t m = 0; m < 3; m++) {
        uart_tmc2209_put_byte(message[m], &message[3]);
    }
    uart_tmc2209_put_byte(message[3], NULL);

    uint8_t reply_message[8];
    for (int i = 0; i < 8; i++) {
        reply_message[i] = uart_tmc2209_get_byte();
    }

    bool valid = validate_reply(reply_message, reg);
    if (valid) {
        *value  = reply_message[3] << 24;
        *value |= reply_message[4] << 16;
        *value |= reply_message[5] << 8;
        *value |= reply_message[6];
    }
    sleep_ms(1);
    return valid;
}

void write_register(uint8_t reg, uint32_t value) {

    uint8_t wire_address = reg & 0x7;
    uint8_t message[8] = { preamble, 0x00, wire_address | 0x80
                         , (value & 0xff000000) >> 24
                         , (value & 0x00ff0000) >> 16
                         , (value & 0x0000ff00) >> 8
                         , (value & 0x000000ff)
                         , 0 };

    for (size_t m = 0; m < 7; m++) {
        uart_tmc2209_put_byte(message[m], &message[7]);
    }
    uart_tmc2209_put_byte(message[7], NULL);

    sleep_ms(1);
}

bool accounted_write(uint8_t reg, uint32_t value) {

    uint32_t dgram_value;
    bool read_valid = read_register(IFCNT, &dgram_value);
    uint8_t dgrams = dgram_value & 0xff;
    if (read_valid) {
        write_register(reg, value);
    }
    read_valid |= read_register(IFCNT, &dgram_value);
    if (read_valid) {
        uint8_t dgrams_after = dgram_value & 0xff;
        return ++dgrams == dgrams_after;
    }
    return false;
}

void assert_stepper_api() {
    uint32_t value;
    if (read_register(IOIN, &value)) {
        assert((value & 0xff000000) == 0x21000000);
    } else {
        assert(!"read failure");
    }
}