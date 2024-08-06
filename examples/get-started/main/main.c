#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "lvgl.h"

#include "bsp/display.h"

#include "examples/lv_examples.h"

static const char *TAG = "app_main";

void app_lvgl_display(void)
{
    bsp_display_lock(0);
    lv_example_get_started_1();
    bsp_display_unlock();
}

void app_main(void)
{
    if (bsp_display_start() == NULL) {
        ESP_LOGE(TAG, "display start failed!");
        abort();
    }
    app_lvgl_display();
    bsp_display_backlight_on();
    ESP_LOGI(TAG, "display start");
}
