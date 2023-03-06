/**
 *
 */
#include <stdio.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <freertos/task.h>
#include <driver/timer.h>
#include "freertos/FreeRTOS.h"
#include <freertos/task.h>
#include <stdbool.h>

#define _KP
#define DEBOUNCE_PERIOD_MS 150
#define MAX_BUFFER_SIZE 21
#define BUFFER_TIMEOUT 2
#define TIMER_DEVIDER 16


typedef struct {
	char elements[MAX_BUFFER_SIZE];
	uint8_t occupied;
} keypad_buffer_t;


char poll_keypad(uint8_t keypad_address);

void init_timer(keypad_buffer_t* buffer);

void clear_buffer(keypad_buffer_t *buffer);

void add_to_buffer(keypad_buffer_t* buffer,char val);
