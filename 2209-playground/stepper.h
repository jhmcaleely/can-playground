#ifndef stepper_h
#define stepper_h

extern const bool away;
extern const bool toward;

void init_stepper();
void enable_motor();
void disable_motor();
void dance();
void move(bool dir, int distance, int delay);
void step_motor();
void rotate(bool direction);

enum TMC2209registers {
    GCONF = 0x00,
    IFCNT = 0x02,
    IOIN = 0x06,
    TCOOLTHRS = 0x14,
    SGTHRS = 0x40,
    SG_RESULT = 0x41,
};

bool read_register(uint8_t reg, uint32_t* value);
void write_register(uint8_t reg, uint32_t value);
bool accounted_write(uint8_t reg, uint32_t value);
void assert_stepper_api();

#endif