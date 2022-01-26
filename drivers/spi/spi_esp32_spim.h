/*
 * Copyright (c) 2020 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SPI_ESP32_SPIM_H_
#define ZEPHYR_DRIVERS_SPI_ESP32_SPIM_H_

#define SPI_MASTER_FREQ_8M      (APB_CLK_FREQ/10)
#define SPI_MASTER_FREQ_9M      (APB_CLK_FREQ/9)    /* 8.89MHz */
#define SPI_MASTER_FREQ_10M     (APB_CLK_FREQ/8)    /* 10MHz */
#define SPI_MASTER_FREQ_11M     (APB_CLK_FREQ/7)    /* 11.43MHz */
#define SPI_MASTER_FREQ_13M     (APB_CLK_FREQ/6)    /* 13.33MHz */
#define SPI_MASTER_FREQ_16M     (APB_CLK_FREQ/5)    /* 16MHz */
#define SPI_MASTER_FREQ_20M     (APB_CLK_FREQ/4)    /* 20MHz */
#define SPI_MASTER_FREQ_26M     (APB_CLK_FREQ/3)    /* 26.67MHz */
#define SPI_MASTER_FREQ_40M     (APB_CLK_FREQ/2)    /* 40MHz */
#define SPI_MASTER_FREQ_80M     (APB_CLK_FREQ/1)    /* 80MHz */

struct spi_esp32_config {
	spi_dev_t *spi;
	const struct device *clock_dev;
	int duty_cycle;
	int input_delay_ns;
	int irq_source;
	bool use_iomux;

	clock_control_subsys_t clock_subsys;

	struct {
		int miso_s;
		int mosi_s;
		int sclk_s;
		int csel_s;
	} signals;

	struct {
		int miso;
		int mosi;
		int sclk;
		int csel;
	} pins;
};

struct spi_esp32_data {
	struct spi_context ctx;
	spi_hal_context_t hal;
	spi_hal_timing_conf_t timing_config;
	spi_hal_dev_config_t dev_config;
	spi_hal_trans_config_t trans_config;
	uint8_t dfs;
	int irq_line;
};

#endif /* ZEPHYR_DRIVERS_SPI_ESP32_SPIM_H_ */
