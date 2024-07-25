idf_component_register(
    SRCS "src/bsp_generic.c" "src/esp_lcd_panel_st7735.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver
    PRIV_REQUIRES esp_lcd
)
