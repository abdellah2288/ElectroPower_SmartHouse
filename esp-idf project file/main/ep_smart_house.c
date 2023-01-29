#include <stdio.h>
#include "ep_wifi.h"
#include "analog_read.h"
#include "freertos/portmacro.h"
void app_main(void)
{

    access_point_t ap =
    {
        .ssid = "Monster",
        .password = "Petra+123"
    };
    init_wifi('s',ap);
    while(1)
    {
        printf("[reading]> %d \n",read_analog_raw(3));
        vTaskDelay(500 / portTICK_PERIOD_MS );
    }
}
