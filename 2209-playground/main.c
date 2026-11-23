#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"

#include "stepper.h"

#define SYSTEM_CLOCK_RATE  128000

int main()
{
    set_sys_clock_khz(SYSTEM_CLOCK_RATE, true);
    stdio_init_all();
    init_stepper();

    printf("Init complete\n");
    
    enable_motor();

    rotate(away);
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

        dance();

        disable_motor();


        sleep_ms(1000);
    }
}
