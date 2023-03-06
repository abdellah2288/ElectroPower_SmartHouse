#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <regex.h>
#include <ep_wifi.h>
#include <mqtt_client.h>
#include <freertos/task.h>
#include <ep_lcd.h>
#include <rc522.h>
#include <pca9685.h>
#include <analog_read.h>
#include "DHT.h"
#ifndef _KP
#include "ep_keypad.h"
#endif

#define FRONT_DOOR_PASSWD "56709"
#define LCD_1_ADDRESS 0x23
#define LCD_0_ADDRESS 0x27
#define SENSOR_PCF_ADDRESS 0x21
#define KEYPAD_ADDRESS 0x20

keypad_buffer_t	buffer;
/*
 * Task handles
 */
TaskHandle_t reading_task;
TaskHandle_t door_task;
TaskHandle_t mqtt_task;
TaskHandle_t lights_task;
TaskHandle_t override_led_vals_task;
/*
 * MQTT client handle
 */
esp_mqtt_client_handle_t client;

volatile esp_mqtt_event_t mqtt_event;

/*
 * Queue in which sensor reading values from the pcf are shown
 */
QueueHandle_t sensor_readings_q;
/*
 * Regular expression var for finding patterns in strings
 */
regex_t reglr;
/**
 * Custom welcome messages
 */
char* welcome_messages[4] ={"-Not at home-","---Busy---","--Coming--","----Welcome----"};
char prev_msg[25]="";
char prev_wlc_msg[25]="----Welcome----";
static rc522_handle_t scanner;
/*
 * Conditional variable to check for MQTT readiness, is set to 1 if the client manages to get a stable connection to the broker
 */
volatile int ready = 0;
volatile bool wlc_msg_displayed = false;

volatile uint8_t led_stats = 0x00;
/*
 * Override byte: overrides normal functioning parameters in favor of instructions recieved via mqtt
 * lower 4 bits are reserved for LED values
 * higher 4 bits are reserved for servos, with the most significant bit handeling the front door
 */
volatile uint8_t override_byte=0x00;



/**
 *@brief reads sensor values and sends to sensor_readings_q queue
 */
void read_pcf8574(void* params);

/**
 * @brief handles data recieved through MQTT
 */
void mqtt_handle_data(void* param);

/**
 * @brief debugging function used to display pcf values in the serial console
 */
void print_pcf_vals(void* params);

/**
 * @brief handles door status according to MQTT instructions and data from the RFID scanner
 */
void handle_doors(void* params);

/**
 * @brief adjusts lights according to mqtt instructions and ldr values
 */
void adjust_lights(void* client);

/**
 * @brief matches string to regular expression
 * @param string string to be matched to
 * @param expression regex expression
 * @return returns match result, 0 if no match was found
 */
int match_to_regex(char * string,char* expression);


/**
 *@brief Reads data from the dht sensors, publishes it to the mqtt broker then display it on lcd_1
 *@brief Also handles mqtt subscriptions when ready flag is set to 1 by the mqtt event handler
 *@brief Reconnects the mqtt client if not ready
 *@param client a pointer to the mqtt client instance
 */
void mqtt_comms(void* client);

void keypad_handler(void* params);

