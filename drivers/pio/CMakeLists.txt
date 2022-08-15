zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PIO_RPI_PICO pio_rpi_pico.c)
