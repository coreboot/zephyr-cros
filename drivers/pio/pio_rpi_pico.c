#include <zephyr/devicetree.h>

struct {
} rpi_pico_pio_data;

struct {

} rpi_pico_pio_config;

#define RPI_PICO_PIO_INIT(idx)                                                      \
        static struct rpi_pico_pio_data rpi_pico_pio_data_##idx = {                     \
        };                                                                      \
        static struct rpi_pico_pio_config rpi_pico_pio_config_##idx = {                 \
        };                                                                      \
        DEVICE_DT_INST_DEFINE(idx, &rpi_pico_pio_init, NULL, &rpi_pico_pio_data_##idx,  \
                              &rpi_pico_pio_config_##idx, POST_KERNEL,              \
                              CONFIG_SPI_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RPI_PICO_PIO_INIT)
