#include "main.h"

TaskHandle_t reading_task;
TaskHandle_t open_door_task;
TaskHandle_t dht_task;
TaskHandle_t ldr_task;
TaskHandle_t override_led_vals_task;

esp_mqtt_client_handle_t client;

QueueHandle_t sensor_readings_q;

static rc522_handle_t scanner;
volatile int ready = 0;

volatile uint8_t led_override_byte=0x00;

void mqtt_event_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
	esp_mqtt_event_t* event = (esp_mqtt_event_t*) event_data;
	switch(event_id)
	{
	case MQTT_EVENT_CONNECTED:
		ready = 1;
		break;
	case MQTT_EVENT_DISCONNECTED:
		ready = 0;
		break;
	case MQTT_EVENT_DATA:
		printf("> event data : %s \n", event->data);
		break;


	}
}

static void rc522_handler(void* arg, esp_event_base_t base, int32_t event_id, void* event_data)
{
    rc522_event_data_t* data = (rc522_event_data_t*) event_data;

    switch(event_id) {
        case RC522_EVENT_TAG_SCANNED: {
                rc522_tag_t* tag = (rc522_tag_t*) data->ptr;
                xTaskGenericNotifyFromISR(open_door_task,0,(tag->serial_number == 1042346802697),eSetValueWithOverwrite ,0,3);
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
			.ssid = "Etudiant",
			.password= ""
	};
	init_wifi('s',ap);



	/*
	 * initialize i2c driver as master
	 */
	i2c_master_init();
	/*
	 * initialize lcd_1 (internal lcd)
	 */
	i2c_lcd_init_sequence(0x23);
	/*
	 * initialize lcd_0 (external lcd)
	 */
	i2c_lcd_init_sequence(0x27);
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
    setFrequencyPCA9685(50);
    turnAllOff();


    /*
     * Configure mqtt client
     */
	esp_mqtt_client_config_t mqtt_cfg = {
		.broker.address.uri = "mqtt://172.16.4.110:1883"
	};
	client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
	esp_mqtt_client_start(client);
	/*
	 * mqtt subscriptions
	 */
	esp_mqtt_client_subscribe(client,"room_1/lights");
	esp_mqtt_client_subscribe(client,"room_2/lights");
	esp_mqtt_client_subscribe(client,"room_3/lights");
	esp_mqtt_client_subscribe(client,"exterior/lights");
	/*
	 * Create queues
	 */
	sensor_readings_q = xQueueCreate(5,1);

	/*
	 * Create tasks
	 * avoid using core 0 as it causes instability, core 1 already has many cycles to spare
	 */
	xTaskCreatePinnedToCore(open_door,"opens the door",2048,NULL,3,&open_door_task,1);
    xTaskCreatePinnedToCore(read_pcf8574,"read from pcf8574",4098,NULL,2,&reading_task,1);
	xTaskCreatePinnedToCore(read_dht_stats,"Read dhts",4098,(void*) &client,3,&dht_task,1);
	xTaskCreatePinnedToCore(read_ldr_vals,"read ldrs",2048,NULL,1,&ldr_task,1);


}

void open_door(void* params)
{
	uint32_t notification_value = 0;
	while(1)
	{
	if(xTaskGenericNotifyWait(0,0,0xffffffffUL,&notification_value,1000/portTICK_PERIOD_MS)==pdPASS)
	{
		switch(notification_value)
		{
		case 1:
			i2c_lcd_write_message("Access Granted!",0x27);
			setPWM(8,0,550);
			setPWM(3,0,550);
			vTaskDelay(3000/portTICK_PERIOD_MS);
			setPWM(3,0,150);
			setPWM(8,0,0);
			i2c_lcd_clear_screen(0x27);
			break;
		case 0:
			i2c_lcd_write_message("Access Denied",0x27);
			setPWM(11,0,550);
			vTaskDelay(1000/portTICK_PERIOD_MS);
			setPWM(11,0,0);
			i2c_lcd_clear_screen(0x27);
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
	//uint8_t clear_pullup = 0b11111111;
	// writes 0b11111111 to pcf8574 before reading, which disables the pull high resistor on all pins
	//i2c_master_write_to_device(I2C_NUM_0,0x27, &clear_pullup,8,100);
	i2c_master_read_from_device(I2C_NUM_0, 0x27, &data, 8, 100);
	xQueueSend(sensor_readings_q,(void*) &data, 10 );
	vTaskDelay(200/portTICK_PERIOD_MS);
	}
}

void read_ldr_vals(void* params)
{

	int pca_pins[] = {12,6,5,4};
	int adc1_channels[] = {5,4,6,7};

	while(1)
	{
	for(int i=0; i<4;i++)
	{
		float reading = (read_analog_raw(adc1_channels[i]) *100) / 3800;
		switch( reading > 50  || (led_override_byte << i) & 0x01)
		{
			case 1:
				setPWM(pca_pins[i],((i==0) ? 550: 0),0);
				break;
			case 0:
				setPWM(pca_pins[i],((i==0) ? 0: 550),0);
				break;
		}
	}
	vTaskDelay(2000/portTICK_PERIOD_MS);
	}
	}

void detect_movement(void* params)
{
	while(1){

	uint8_t data=0x00;

	uint8_t pin_select=0x80 ;

	if(xQueueReceive(sensor_readings_q,(uint8_t *) &data,500 * portTICK_PERIOD_MS) == pdTRUE)
	{

		for(int i=0;i<4;i++)
			{
			if(!!( (data<<i) & pin_select ) )
				{
				gpio_set_level(4,1);
				printf("> mov %d \n",i);
				vTaskDelay(50/portTICK_PERIOD_MS);
				gpio_set_level(4,0);
				}
			}
	}}
}

void read_dht_stats(void* params)
{
	esp_mqtt_client_handle_t cli =*((esp_mqtt_client_handle_t *) params) ;
	while(1)
	{

	int16_t temperature =0,humidity=0;
	static char temp_data[10];
	static char data_str[23];
	dht_read_data(0, GPIO_NUM_14, &humidity,  &temperature);
	sprintf(data_str,"Temperature: %d C°",(temperature/10));

	i2c_lcd_send_command(0,0x01,0x23);
	vTaskDelay(500/portTICK_PERIOD_MS);
	i2c_lcd_write_message(data_str,0x23);

	if(ready)
	{
		esp_mqtt_client_subscribe(cli,"room_1/lights",0);
		sprintf(temp_data,"%d C°",(temperature/10));
		esp_mqtt_client_publish(cli,"room_1/temperature",temp_data,10,1,1);

		dht_read_data(0, GPIO_NUM_27, &humidity,  &temperature);
		sprintf(temp_data,"%d C°",(temperature/10));
		esp_mqtt_client_publish(cli,"room_2/temperature",temp_data,10,1,1);
	}
	else esp_mqtt_client_reconnect(cli);

	vTaskDelay(10000/portTICK_PERIOD_MS);
	}
}

