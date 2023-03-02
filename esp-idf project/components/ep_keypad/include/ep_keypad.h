#include <stdio.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
char keypad_array[][3] = {"123","456","789","*0#"};

void poll_keypad(uint8_t keypad_address);
