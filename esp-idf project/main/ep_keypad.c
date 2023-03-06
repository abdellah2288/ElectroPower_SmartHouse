#ifndef _KP
#include "ep_keypad.h"
#endif

char keypad_array[4][3] = {"123","456","789","*0#"};





void clear_buffer(keypad_buffer_t * buffer)
{
	buffer->occupied = 0;
	for(int i = 0; i< MAX_BUFFER_SIZE; i++) (buffer->elements)[i] = '\0';
}

void add_to_buffer(keypad_buffer_t* buffer,char val)
{

	if((buffer->occupied) >= (MAX_BUFFER_SIZE - 2)) clear_buffer(buffer);
	(buffer->elements)[buffer->occupied] = val;
	buffer->occupied += 1;
}



void init_timer(keypad_buffer_t* buffer)
{
	timer_config_t timer_conf =
	{
			.divider = TIMER_DEVIDER,
			.counter_en = true,
			.alarm_en = false,
			.auto_reload =false,
			.clk_src = TIMER_SRC_CLK_APB,
	};
	timer_init(TIMER_GROUP_0, TIMER_0, &timer_conf);
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);

	timer_start(TIMER_GROUP_0, TIMER_0);
}


char poll_keypad(uint8_t keypad_address)
{
		uint8_t data=0x00;
		uint8_t activate = 0xf0;
		uint8_t lines = 0;
		uint8_t cols = 0;
		i2c_master_write_to_device(I2C_NUM_0,keypad_address, &activate,1,100);
		i2c_master_read_from_device(I2C_NUM_0, keypad_address, &data, 1, 100);
		switch((data ^ 0xff) >> 4)
		{
			case 8:
				lines = 1;
				break;
			case 4:
				lines = 2;
				break;
			case 2:
				lines = 3;
				break;
			case 1:
				lines = 4;
				break;
		}

		activate = 0x0f;
		i2c_master_write_to_device(I2C_NUM_0,keypad_address, &activate,1,100);
		i2c_master_read_from_device(I2C_NUM_0, keypad_address, &data, 1, 100);
		switch(((data ^ 0xff) & 0x07))
		{
			case 4:
				cols = 1;
				break;
			case 2:
				cols = 2;
				break;
			case 1:
				cols = 3;
				break;
		}
		//printf("[lines][cols]> %d %d \n",lines,cols);
		if(lines && cols)
		{
			vTaskDelay(DEBOUNCE_PERIOD_MS/portTICK_PERIOD_MS);
			return(keypad_array[lines-1][cols-1]);
		}
		return '\0';
}
