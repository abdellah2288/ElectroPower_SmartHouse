#include "main.h"
#include "ep_keypad.h"



void mqtt_event_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
	switch(event_id)
	{
	case MQTT_EVENT_CONNECTED:
		ready = 1;
		break;
	case MQTT_EVENT_DISCONNECTED:
		ready = 0;
		break;
	case MQTT_EVENT_DATA:
		mqtt_event = *((esp_mqtt_event_t *) event_data);
		xTaskGenericNotifyFromISR(mqtt_task,0,0,eNoAction,0,2);
		break;
	}
}

static void rc522_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
    rc522_event_data_t* data = (rc522_event_data_t*) event_data;

    switch(event_id) {
        case RC522_EVENT_TAG_SCANNED: {
                rc522_tag_t* tag = (rc522_tag_t*) data->ptr;
                //printf("> %" PRIx64 "\n",tag->serial_number);
                xTaskGenericNotifyFromISR(door_task,0,((tag->serial_number == 1042346802697) | (tag->serial_number == 209704347857) << 1),eSetValueWithOverwrite ,0,3);
            }
            break;
    }
}

void app_main(void)
{
	/*
	 * Reset JTAG pins so they can be used for normal GPIO operations
	 */
	gpio_reset_pin(GPIO_NUM_15);
	gpio_reset_pin(GPIO_NUM_14);
	gpio_reset_pin(GPIO_NUM_12);
	/*
	 * init wifi, change ap info according to used network
	 */
	access_point_t ap =
	{
			.ssid = "Orange-1036",
			.password= "1FRDD7HJ9AH"
	};
	init_wifi('s',ap);



	/*
	 * initialize i2c driver as master
	 */
	i2c_master_init();
	/*
	 * initialize lcd_1 (internal lcd)
	 */
	i2c_lcd_init_sequence(LCD_1_ADDRESS);
	/*
	 * initialize lcd_0 (external lcd)
	 */
	i2c_lcd_init_sequence(LCD_0_ADDRESS);
    /*
     * Configure rfid gpio and initialize rc522 scanner
     */
	rc522_config_t config = {
        .spi.host = VSPI_HOST,
        .spi.miso_gpio = 19,
        .spi.mosi_gpio = 23,
        .spi.sck_gpio = 18,
        .spi.sda_gpio = 26,
    };

    rc522_create(&config, &scanner);
    rc522_register_events(scanner, RC522_EVENT_ANY, rc522_handler, NULL);
    rc522_start(scanner);

    /*
     * Configure pca9685 and set all pins to off
     */
    set_pca9685_adress(0x40);
    resetPCA9685();
    setFrequencyPCA9685(60);
    turnAllOff();

  
    /*
     * Configure mqtt client
     */
	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = "mqtt://192.168.1.106:1883"
	};
	client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
	esp_mqtt_client_start(client);



	/*
	 * Create queues
	 */
	sensor_readings_q = xQueueCreate(5,1);
/*
	 * Create tasks
	 * avoid using core 0 as it causes instability, core 1 already has many cycles to spare
	 */
	xTaskCreatePinnedToCore(handle_doors,"opens the door",2048,NULL,3,&door_task,1);
    //xTaskCreatePinnedToCore(read_pcf8574,"read from pcf8574",4098,NULL,2,&reading_task,1);
	xTaskCreatePinnedToCore(mqtt_comms,"MQTT communications",4098,(void*) &client,3,NULL,1);
	xTaskCreatePinnedToCore(adjust_lights,"adjust lights",2048,&client,1,&lights_task,1);
	xTaskCreatePinnedToCore(mqtt_handle_data,"MQTT data",2048,NULL,2,&mqtt_task,1);
	xTaskCreatePinnedToCore(keypad_handler,"keypad handler",4098,NULL,2,NULL,0);
	//xTaskCreatePinnedToCore(print_pcf_vals,"print from pcf",2048,NULL,1,NULL,1);

}

void keypad_handler(void* params)
{
	char c = '\0';
	uint8_t clear_pullup = 0xff;
	double prev_time=0;
	double curr_time=0;

	i2c_master_write_to_device(I2C_NUM_0,KEYPAD_ADDRESS, &clear_pullup,1,100);
	init_timer(&buffer);

	while(1)
    {

		timer_get_counter_time_sec(TIMER_GROUP_0,TIMER_0,&curr_time);

		c = poll_keypad(KEYPAD_ADDRESS);

		if((prev_time - curr_time)  > 10) clear_buffer(&buffer);


		switch(c)
		{
			case '*':
				/*	Delete last inserted character	*/
				buffer.occupied -= 1;
				(buffer.elements)[buffer.occupied] = '\0';
				prev_time = curr_time;
				break;
			case '#':
				/*	submit buffer	*/
				xTaskGenericNotify(door_task,0,!(strcmp(buffer.elements,FRONT_DOOR_PASSWD)),eSetValueWithOverwrite,0);
				clear_buffer(&buffer);
				prev_time = curr_time;
				break;
			case '\0':
				break;
			default:
				prev_time = curr_time;
				add_to_buffer(&buffer,c);
				break;

		}

    	vTaskDelay(50/portTICK_PERIOD_MS);
    }
}
void handle_doors(void* params)
{
	/*
	 * It's okay to store the notification value in an 8 bit int (Check the values being sent)
	 */
	uint8_t notification_value = 0;

	while(1)
	{
	/*Check if an MQTT open door request was sent, if not default to a closed door (duh)*/
	if(override_byte & 0x80) setPWM(DOOR_SERVO,0,500);
	else setPWM(DOOR_SERVO,0,50);
	
	if(override_byte & 0x40) setPWM(GARAGE_SERVO,0,50);
	else setPWM(GARAGE_SERVO,0,500);
	
	if(xTaskGenericNotifyWait(0,0,0xffffffffUL,&notification_value,1000/portTICK_PERIOD_MS)==pdPASS)
	{
		switch(notification_value)
		{
		case 2:
			i2c_lcd_clear_screen(LCD_0_ADDRESS);
			vTaskDelay(400/portTICK_PERIOD_MS);
			
			i2c_lcd_write_message("Access Granted!",LCD_0_ADDRESS);
			
			setPWM(GARAGE_LED_GREEN,0,550);
			setPWM(GARAGE_SERVO,0,50);
			
			vTaskDelay(3000/portTICK_PERIOD_MS);
			
			setPWM(GARAGE_SERVO,0,500);
			setPWM(GARAGE_LED_GREEN,0,0);
			
			i2c_lcd_clear_screen(LCD_0_ADDRESS);

			break;
		case 1:
			
			i2c_lcd_clear_screen(LCD_0_ADDRESS);
			vTaskDelay(400/portTICK_PERIOD_MS);
			i2c_lcd_write_message("Access Granted!",LCD_0_ADDRESS);
			
			setPWM(DOOR_LED_GREEN,0,550);
			
			setPWM(DOOR_SERVO,0,500);
			
			vTaskDelay(3000/portTICK_PERIOD_MS);
			
			setPWM(DOOR_SERVO,0,50);
			
			setPWM(DOOR_LED_GREEN,0,0);
			
			i2c_lcd_clear_screen(LCD_0_ADDRESS);
			
			wlc_msg_displayed = false;
			
			break;
		default:
			i2c_lcd_clear_screen(LCD_0_ADDRESS);
			/*Wait for clear command to settle*/
			vTaskDelay(400/portTICK_PERIOD_MS);
			i2c_lcd_write_message("Access Denied",LCD_0_ADDRESS);

			setPWM(GARAGE_LED_RED,0,550);
			setPWM(DOOR_LED_RED,0,550);

			setPWM(GARAGE_BUZZER,0,900);
			setPWM(DOOR_BUZZER,0,900);

			vTaskDelay(1000/portTICK_PERIOD_MS);

			setPWM(GARAGE_LED_RED,0,0);
			setPWM(DOOR_LED_RED,0,0);
			
			setPWM(GARAGE_BUZZER,0,0);
			setPWM(DOOR_BUZZER,0,0);
			
			i2c_lcd_clear_screen(LCD_0_ADDRESS);
			
			wlc_msg_displayed = false;
			
			break;
		}

	}
	}
}


void read_pcf8574(void* params)
{
	while(1)
	{
	uint8_t data=0x00;
	uint8_t clear_pullup = 0xff;
	// writes 0b11111111 to pcf8574 before reading, which disables the pull high resistor on all pins
	//i2c_master_write_to_device(I2C_NUM_0,SENSOR_PCF_ADDRESS, &clear_pullup,8,100);
	i2c_master_read_from_device(I2C_NUM_0, SENSOR_PCF_ADDRESS, &data, 8, 100);
	xQueueSend(sensor_readings_q,(void*) &data, 10 );
	vTaskDelay(200/portTICK_PERIOD_MS);
	}
}

void adjust_lights(void* client)
{

	int pca_pins[] = {EXTERNAL_LEDS,ROOM1_LEDS,ROOM2_LEDS,ROOM3_LEDS};
	int adc1_channels[] = {5,4,6,7};
	esp_mqtt_client_handle_t cli =*((esp_mqtt_client_handle_t *) client) ;
	while(1)
	{
	for(int i=0; i<4;i++)
	{
		float reading = (i==0)? 100 - (read_analog_raw(adc1_channels[i]) *100) / 3800 : (read_analog_raw(adc1_channels[i]) *100) / 3800;
		int mqtt_status= ((override_byte) & (0x01 << i));
		int curr_status = (led_stats & (1<<i));

		if( ( (reading < 50)  || mqtt_status )   )
		{
			if(!curr_status)
			{
			setPWM(pca_pins[i], 550,0);
			led_stats = led_stats | (1<<i);
			}
		}
		else
		{
			setPWM(pca_pins[i], 0,0);
			led_stats = led_stats & ((1<<i)^0xff);

		}
	}
	vTaskDelay(2000/portTICK_PERIOD_MS);
	}
	}

void print_pcf_vals(void* params)
{
	while(1){

	uint8_t data=0x00;

	uint8_t pin_select=0x01 ;

	if(xQueueReceive(sensor_readings_q,(uint8_t *) &data,500 * portTICK_PERIOD_MS) == pdTRUE)
	{

		for(int i=0;i<8;i++)
			{
			if( !((data>>i) & pin_select)  )
				{
				vTaskDelay(500/portTICK_PERIOD_MS);
				}
			vTaskDelay(500/portTICK_PERIOD_MS);
			}

	}}
}

void mqtt_comms(void* client)
{
	esp_mqtt_client_handle_t cli =*((esp_mqtt_client_handle_t *) client) ;
	while(1)
	{

	int16_t temperature =0,humidity=0;
	static char temp_data[16];
	static char data_str[23];
	dht_read_data(0, GPIO_NUM_14, &humidity,  &temperature);
	sprintf(data_str,"Temperature: %d C°",(temperature/10));

	i2c_lcd_send_command(0,0x01,LCD_1_ADDRESS);
	vTaskDelay(500/portTICK_PERIOD_MS);
	i2c_lcd_write_message(data_str,LCD_1_ADDRESS);

	if(ready)
	{
		/*
		 * subscriptions
		 */
		esp_mqtt_client_subscribe(cli,"ext_door",0);
		esp_mqtt_client_subscribe(cli,"room_1/lights",0);
		esp_mqtt_client_subscribe(cli,"room_2/lights",0);
		esp_mqtt_client_subscribe(cli,"room_3/lights",0);
		esp_mqtt_client_subscribe(cli,"exterior/lights",0);
		esp_mqtt_client_subscribe(cli,"wlc_msg",1);
		/*
		 * Room 1 stats
		 */
		sprintf(temp_data,"%d C°",(temperature/10));
		if(temperature) esp_mqtt_client_publish(cli,"room_1/temperature",temp_data,10,1,1);
		sprintf(temp_data,"%d %%",humidity/10);
		if(humidity) esp_mqtt_client_publish(cli,"room_1/humidity",temp_data,4,1,1);
		/*
		 * Room 3 stats
		 */
		dht_read_data(0, GPIO_NUM_27, &humidity,  &temperature);
		sprintf(temp_data,"%d C°",(temperature/10));
		esp_mqtt_client_publish(cli,"room_3/temperature",temp_data,10,1,1);
		sprintf(temp_data,"%d %%",(humidity/10));
		esp_mqtt_client_publish(cli,"room_3/humidity",temp_data,4,1,1);
	}
	else esp_mqtt_client_reconnect(cli);

	vTaskDelay(10000/portTICK_PERIOD_MS);
	}
}

void mqtt_handle_data(void* params)
{

	while(1){

			if (xTaskGenericNotifyWait(0,0,0xffffffffUL,NULL,1000/portTICK_PERIOD_MS)==pdPASS) {

				switch(*((mqtt_event.data) + 1))
				{
				case 'l':
						int room = *(mqtt_event.data + 2) -'0';
						int status = !(*(mqtt_event.data) -'o');
						override_byte = ((override_byte & (1 << room)) & (status << room)) ? override_byte : override_byte ^ (1<<room) ;
						break;
				case 'd':
					if((*(mqtt_event.data) -'o') ^ !(override_byte & 0x80)) override_byte ^= 0x80;

					break;
				case 'g':
					if((*(mqtt_event.data) -'o') ^ !(override_byte & 0x40)) override_byte ^= 0x40;
					break;
				default:
					if(strcmp(welcome_messages[(*(mqtt_event.data) -'0')],prev_wlc_msg))
					{
					strcpy(prev_wlc_msg,welcome_messages[(*(mqtt_event.data) -'0')]);
					i2c_lcd_clear_screen(LCD_0_ADDRESS);
					vTaskDelay(400/portTICK_PERIOD_MS);
					i2c_lcd_write_message(welcome_messages[(*(mqtt_event.data) -'0')],LCD_0_ADDRESS);
					wlc_msg_displayed = true;
					}
					break;
				}
			}
			if(buffer.occupied != 0 && strcmp(prev_msg,buffer.elements))
			{
				strcpy(prev_msg,buffer.elements);
				i2c_lcd_clear_screen(LCD_0_ADDRESS);
				vTaskDelay(200/portTICK_PERIOD_MS);
				i2c_lcd_write_message(buffer.elements,LCD_0_ADDRESS);
			}
			else if( !wlc_msg_displayed && buffer.occupied == 0)
			{
				i2c_lcd_clear_screen(LCD_0_ADDRESS);
				vTaskDelay(200/portTICK_PERIOD_MS);
				i2c_lcd_write_message(prev_wlc_msg,LCD_0_ADDRESS);
				wlc_msg_displayed = true;
			}
			vTaskDelay(200/portTICK_PERIOD_MS);
		}
}
/*
 * Misc functions
 */
int match_to_regex(char * string,char* expression)
{
	regcomp( &reglr, expression, 0);
	return !(regexec(&reglr, string, 0, NULL, 0));
}

