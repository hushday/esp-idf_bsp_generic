#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include "driver/i2c_master.h"

#include "bsp/display.h"
#include "esp_lvgl_port.h"

// Pinout

// I2C
#define BSP_I2C_SCL (CONFIG_BSP_I2C_GPIO_SCL)
#define BSP_I2C_SDA (CONFIG_BSP_I2C_GPIO_SDA)

// LCD
#define BSP_LCD_DATA0 (CONFIG_BSP_DISPLAY_MOSI_GPIO)
#define BSP_LCD_PCLK (CONFIG_BSP_DISPLAY_SCLK_GPIO)
#define BSP_LCD_CS (CONFIG_BSP_DISPLAY_CS_GPIO)
#define BSP_LCD_DC (CONFIG_BSP_DISPLAY_DC_GPIO)
#define BSP_LCD_RST (CONFIG_BSP_DISPLAY_RST_GPIO)
#define BSP_LCD_BACKLIGHT (CONFIG_BSP_DISPLAY_BACKLIGHT_GPIO)

typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;
    uint32_t buffer_size;
    bool double_buffer;
    struct {
        unsigned int buff_dma : 1;
        unsigned int buff_spiram : 1;
    } flags;
} bsp_display_cfg_t;

// I2C
#define BSP_I2C_NUM CONFIG_BSP_I2C_NUM

esp_err_t bsp_i2c_init(void);
esp_err_t bsp_i2c_deinit(void);

extern i2c_master_bus_handle_t bsp_i2c_bus_handle;

// LCD
#define BSP_LCD_PIXEL_CLOCK_HZ (CONFIG_BSP_DISPLAY_PIXEL_CLOCK * 1000 * 1000)
#define BSP_LCD_SPI_NUM (SPI2_HOST)

lv_display_t *bsp_display_start(void);
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);
lv_indev_t *bsp_display_get_input_dev(void);
bool bsp_display_lock(uint32_t timeout_ms);
void bsp_display_unlock(void);
void bsp_display_rotate(lv_disp_t *disp, lv_display_rotation_t rotation);

#ifdef __cplusplus
}
#endif
