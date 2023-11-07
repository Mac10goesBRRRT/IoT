#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#define LED1 5 // port pin of on - board LED
#define LED2 4
#define LED3 26

void blink(void * param);

void app_main()
{
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT); // set GPIO as output
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT); // set GPIO as output
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT); // set GPIO as output

    static uint16_t leuchte1[] = {4, 1000};
    TaskHandle_t xHandle1 = NULL;
    xTaskCreate(&blink, "blink1", 4000, &leuchte1, tskIDLE_PRIORITY, &xHandle1);
    configASSERT(xHandle1);

    static uint16_t leuchte2[] = {5, 100};
    TaskHandle_t xHandle2 = NULL;
    xTaskCreate(&blink, "blink2", 4000, &leuchte2, tskIDLE_PRIORITY, &xHandle2);
    configASSERT(xHandle2);

    static uint16_t leuchte3[] = {26, 10};
    TaskHandle_t xHandle3 = NULL;
    xTaskCreate(&blink, "blink3", 4000, &leuchte3, tskIDLE_PRIORITY, &xHandle3);
    configASSERT(xHandle3);
}


void blink(void *param)
{
    bool state = 1;
    uint16_t* localarray = (uint16_t*)param;
    for (;;)
    {
        gpio_set_level(localarray[0], state);
        vTaskDelay(localarray[1] / portTICK_PERIOD_MS);
        state = !state;
        //gpio_set_level(localarray[0], 0);
        //vTaskDelay(localarray[1] / portTICK_PERIOD_MS);
    }
}