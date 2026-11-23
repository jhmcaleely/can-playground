#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"

#include "stepper.h"

#define SYSTEM_CLOCK_RATE  128000

//                             INTERFACE PARAMETERS
//
// GPIO pins
#define CAN_TX           2
#define LED_PIN          25
#define TRANSCIEVER_SLNT 4

int main()
{
    set_sys_clock_khz(SYSTEM_CLOCK_RATE, true);
    stdio_init_all();

    // Initialize LED
    gpio_init(LED_PIN) ;
    gpio_set_dir(LED_PIN, GPIO_OUT) ;
    gpio_put(LED_PIN, 0) ;

    // Power off CAN transciever (avoids transients on bus)
    gpio_init(TRANSCIEVER_SLNT) ;
    gpio_set_dir(TRANSCIEVER_SLNT, GPIO_OUT) ;
    gpio_put(TRANSCIEVER_SLNT, 1) ;


    init_stepper();

    printf("Init complete\n");
    
    enable_motor();

    gpio_put(LED_PIN, 1) ;
    rotate(away);
    gpio_put(LED_PIN, 0) ;
    rotate(toward);

    disable_motor();

    assert_stepper_api();

    uint32_t reg;
    if (read_register(GCONF, &reg)) {
        uint32_t gconf = reg & 0x3FF;
        uint32_t pdn_diable = (gconf & 0x40) >> 6;
        printf("GCONF %x, PDN_DISABLE %x\n", gconf, pdn_diable);

        if (accounted_write(GCONF, gconf | 0x40)) {
            printf("written GCONF\n");
        }
    }

    if (read_register(GCONF, &reg)) {
        uint32_t gconf = reg & 0x3FF;
        uint32_t pdn_diable = (gconf & 0x40) >> 6;
        printf("GCONF %x, PDN_DISABLE %x\n", gconf, pdn_diable);
    }

    if (accounted_write(TCOOLTHRS, 64)) {
        printf("TCOOLTHRS: 3200\n");
    }


    while (true) {
        // prove comms continue to work each loop.
        assert_stepper_api();

        enable_motor();

        gpio_put(LED_PIN, 1) ;
        dance();

        disable_motor();
        gpio_put(LED_PIN, 0) ;


        sleep_ms(1000);
    }
}
