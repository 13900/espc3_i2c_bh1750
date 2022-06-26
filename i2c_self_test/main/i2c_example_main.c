#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "bh1750.h"
static const char *TAG = "i2c-example";


void app_main(void)
{
    ESP_ERROR_CHECK(I2C_Init());
    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, NULL, 10, &BH1750_TASK_HANDEL);
}
