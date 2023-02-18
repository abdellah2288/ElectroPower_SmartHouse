#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>

#include <ep_wifi.h>
#include <mqtt_client.h>
#include <freertos/task.h>
#include <ep_lcd.h>
#include <rc522.h>
#include <pca9685.h>
#include <analog_read.h>
#include "DHT.h"


/*
 * queue handle for sensor readings, will suspend publishing task until full
 */

/*
 * array containing room brightness info read from ldrs on channels 5,4,6 and 7
 * Calculated using the formula ( read_analog_raw(ADC1_CHANNEL)* 100) / 3800
 */
int brightness_array[4];

/*
 * @brief calculates the factorial of a given integer
 * @param number integer to calculate the factorial of
 * @disclaimer I didnt bother with making it perfect since it'll just calculate a few specific factorials
 * careful when using it, rewriting would be optimal
 */
int factorial(int number);
/**
 *@brief reads sensor values and sends to sensor_readings_q queue
 */
void read_pcf8574(void* params);
/**
 *@brief publishes to mqtt broker from sensor_readings_q, is blocked until sensor_readings_q is full
 */
void publish_readings(void);

/**
 * @brief initializes mqtt according to the provided broker uri and credentials
 * @param broker_address, broker's uri
 * @param username, mqtt client username
 * @param password, mqtt client password
 */
//void init_mqtt(char* address, char* username,char* password);
void detect_movement(void* params);

void open_door(void* params);

/**
 * @brief reads ldr values from adc channels
 */
void read_ldr_vals(void* params);



/**
 *@brief Reads data from the dht sensors, publishes it to the mqtt broker then display it on lcd_1
 */
void read_dht_stats(void* params);

