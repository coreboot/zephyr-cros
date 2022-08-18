#include <zephyr/devicetree.h>

struct {
} rpi_pico_pio_data;

struct {

} rpi_pico_pio_config;

int rpi_pico_pio_init(const struct device *dev)
{
        struct rpi_pico_pio_data *data = dev->data;
        const struct rpi_pico_pio_config *cfg = dev->config;
        int ret;

        uint32_t offset;
        pio_sm_config sm_config;

        offset = pio_add_program(pio, &uart_tx_program);
        sm_config = pio_get_default_sm_config();

        sm_config_set_sideset(&sm_config, SIDESET_BIT_COUNT, true, false);
        sm_config_set_out_shift(&sm_config, true, false, 0);
        sm_config_set_out_pins(&sm_config, pin_tx, 1);
        sm_config_set_sideset_pins(&sm_config, pin_tx);
        sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&sm_config, div);
        sm_config_set_wrap(&sm_config,
                           offset + RPI_PICO_PIO_GET_WRAP_TARGET(uart_tx),
                           offset + RPI_PICO_PIO_GET_WRAP(uart_tx));

        pio_sm_set_pins_with_mask(pio, TX_STATE_MACHINE, BIT(pin_tx), BIT(pin_tx));
        pio_sm_set_pindirs_with_mask(pio, TX_STATE_MACHINE, BIT(pin_tx), BIT(pin_tx));
        pio_sm_init(pio, TX_STATE_MACHINE, offset, &sm_config);
        pio_sm_set_enabled(pio, TX_STATE_MACHINE, true);

	return ret;
}


#define RPI_PICO_PIO_INIT(idx)                                                      \
        static struct rpi_pico_pio_data rpi_pico_pio_data_##idx = {                     \
        };                                                                      \
        static struct rpi_pico_pio_config rpi_pico_pio_config_##idx = {                 \
        };                                                                      \
        DEVICE_DT_INST_DEFINE(idx, &rpi_pico_pio_init, NULL, &rpi_pico_pio_data_##idx,  \
                              &rpi_pico_pio_config_##idx, POST_KERNEL,              \
                              CONFIG_SPI_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RPI_PICO_PIO_INIT)
