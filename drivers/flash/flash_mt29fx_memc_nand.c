/*
 * Copyright (c) 2023 Schlumberger
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT micron_mt29fx_memc_nand

#include <stdint.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_mt29fx, CONFIG_FLASH_LOG_LEVEL);

#define BAD_MARK_OVERHEAD      1
#define ECC_OVERHEAD_PER_CHUNK 7

#define MAX_DMA_TRANSACTION_SIZE 4095


#define ERASE_VALUE	    0xff

#define NAND_FLASH_ERASE_BLOCK_START_COMMAND   0x60
#define NAND_FLASH_ERASE_BLOCK_STOP_COMMAND    0xD0
#define NAND_FLASH_READ_PAGE_START_COMMAND     0x00
#define NAND_FLASH_READ_PAGE_STOP_COMMAND      0x30
#define NAND_FLASH_PROGRAM_PAGE_START_COMMAND  0x80
#define NAND_FLASH_PROGRAM_PAGE_STOP_COMMAND   0x10
#define NAND_FLASH_RESET_COMMAND               0xff
#define NAND_FLASH_GET_FEATURE_COMMAND         0xee
#define NAND_FLASH_SET_FEATURE_COMMAND         0xef
#define NAND_FLASH_GET_STATUS_COMMAND          0x70
#define NAND_FLASH_CHANGE_WRITE_COLUMN_COMMAND 0x85

#define NAND_FLASH_RANDOM_DATA_READ_START_COMMAND 0x05
#define NAND_FLASH_RANDOM_DATA_READ_STOP_COMMAND  0xe0

#define NAND_FLASH_STATUS_REG_FAIL  BIT(0)
#define NAND_FLASH_STATUS_REG_FAILC BIT(1)
#define NAND_FLASH_STATUS_REG_ARDY  BIT(5) /* Ready/Busy Array */
#define NAND_FLASH_STATUS_REG_RDY   BIT(6) /* Ready/Busy I/O */
#define NAND_FLASH_STATUS_REG_WP    BIT(7)

#define NAND_FLASH_STATUS_REWRITE_RECOMMENDED BIT(3)

#define NAND_FLASH_FEATURE_REG_TIMING_MODE 0x1
#define NAND_FLASH_FEATURE_REG_ARRAY_OPERATIONS 0x90
#define NAND_FLASH_FEATURE_REG_ARRAY_OPERATIONS_ECC_ENABLE BIT(3)

struct dma_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	struct dma_block_config blk_cfg;
	struct k_sem status_sem;
};

struct flash_mt29fx_data {
	struct k_sem sem;
	bool internal_ecc_enabled;
#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
	uint8_t *internal_buffer;
	uint32_t internal_buffer_size;
#endif
#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_DMA
	struct dma_stream dma_read;
	struct dma_stream dma_write;
#endif
};

struct flash_mt29fx_config {
	uint32_t base;
	uint64_t size;
	uint32_t ale_mask;
	uint32_t cle_mask;
	struct flash_pages_layout layout;
	struct flash_parameters parameters;
	struct gpio_dt_spec ready_busy_dt;
	struct gpio_dt_spec write_protect_dt;
	uint16_t write_page_size;
	uint16_t partial_write_page_size;
	uint32_t block_size;
	uint32_t num_blocks;
#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
	uint16_t ecc_chunk_size;
	uint8_t ecc_syndrome_bytes;
	uint16_t oob_size;
	const struct bch_def *bch_config;
#endif
	const struct device *memc_dev;
	uint8_t memc_index;
};

#ifdef CONFIG_FLASH_PAGE_LAYOUT
static void flash_mt29fx_get_page_layout(const struct device *dev,
					 const struct flash_pages_layout **layout,
					 size_t *layout_size)
{
	const struct flash_mt29fx_config *dev_config = dev->config;

	*layout = &dev_config->layout;
	*layout_size = 1;
}
#endif

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_DMA
static void dma_read_cb(const struct device *dma_dev, void *user_data, uint32_t channel, int status)
{
	const struct device *dev = user_data;
	struct flash_mt29fx_data *data = dev->data;

	k_sem_give(&data->dma_read.status_sem);
}

static int dma_init(const struct device *dev)
{
	struct flash_mt29fx_data *dev_data = dev->data;
	const struct flash_mt29fx_config *dev_config = dev->config;

	if (dev_data->dma_read.dma_dev != NULL) {
		struct dma_stream *dma_read = &dev_data->dma_read;
		int ret;

		if (!device_is_ready(dma_read->dma_dev)) {
			return -ENODEV;
		}

		dma_read->blk_cfg.source_address = dev_config->base;

		dma_read->blk_cfg.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
		dma_read->blk_cfg.dest_addr_adj = DMA_ADDR_ADJ_INCREMENT;
		dma_read->blk_cfg.fifo_mode_control = 1;
		dma_read->blk_cfg.bus_lock_enable = 0;
		dma_read->dma_cfg.head_block = &dma_read->blk_cfg;
		dma_read->dma_cfg.user_data = (void *)dev;

		ret = dma_config(dma_read->dma_dev, dma_read->dma_channel,
				 &dma_read->dma_cfg);
		if (ret) {
			return ret;
		}

		k_sem_init(&dma_read->status_sem, 0, 1);
	}

	/* todo */
	/* if (data->dma_write.dma_dev != NULL) { */
	/* } */

	return 0;
}
#endif

static void flash_mt29fx_generate_address_segment(const struct flash_mt29fx_config *dev_config,
						  uint32_t address, uint64_t *address_segment, bool oob_data)
{
	int page_size = dev_config->parameters.write_block_size;
	int leading_zeros = __builtin_clz(page_size - 1) - 16;

	*address_segment = address & (page_size - 1);

	if (oob_data) {
		*address_segment += page_size;
	}

	*address_segment |= (uint64_t)(address & ~(page_size - 1)) << leading_zeros;
	*address_segment = sys_cpu_to_le64(*address_segment);
}

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
static void flash_mt29fx_generate_ecc_column_address(const struct flash_mt29fx_config *dev_config,
						     int ecc_chunk_index, uint16_t *address_segment)
{
	int page_size = dev_config->parameters.write_block_size;

	*address_segment = page_size + ecc_chunk_index * ECC_OVERHEAD_PER_CHUNK + BAD_MARK_OVERHEAD;
}
#endif

static void flash_mt29fx_wait_until_ready(const struct flash_mt29fx_config *dev_config)
{
	if (!dev_config->ready_busy_dt.port) {
		return;
	}

	for (;;) {
		int ret = gpio_pin_get_dt(&dev_config->ready_busy_dt);

		if (ret < 0 || ret == 1) {
			return;
		}
	}
}

static inline void flash_mt29fx_write_command(const struct flash_mt29fx_config *dev_config,
					      uint8_t command)
{
	volatile uint16_t *ahb = (uint16_t *)(dev_config->base + dev_config->cle_mask);
	*ahb = command;
}

static inline void flash_mt29fx_write_address(const struct flash_mt29fx_config *dev_config,
					      uint8_t *addr, int len)
{
	volatile uint16_t *ahb = (uint16_t *)(dev_config->base + dev_config->ale_mask);
	for (int i = 0; i < len; i++) {
		*ahb = addr[i];
	}
}

static inline void flash_mt29fx_write_data(const struct flash_mt29fx_config *dev_config,
					   const uint8_t *data, int len)
{
	volatile uint16_t *addr = (uint16_t *)dev_config->base;
	for (int i = 0; i < len; i++) {
		*addr = data[i];
	}
}

static inline void flash_mt29fx_read_data(const struct device *dev,
					  uint8_t *data, int len)
{
	struct flash_mt29fx_data *dev_data = dev->data;
	const struct flash_mt29fx_config *dev_config = dev->config;
	volatile uint16_t *addr = (uint16_t *)dev_config->base;

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_DMA
	if (dev_data->dma_read.dma_dev != NULL) {
		struct dma_stream *dma_read = &dev_data->dma_read;

		k_sem_reset(&dma_read->status_sem);

		while (len > 4) {
			int dma_len = MIN(len, MAX_DMA_TRANSACTION_SIZE);
			int ret;

			ret = dma_reload(dma_read->dma_dev, dma_read->dma_channel,
					 dev_config->base, (uint32_t)data, dma_len);

			if (ret < 0) {
				break;
			}
			ret = dma_start(dma_read->dma_dev, dma_read->dma_channel);

			if (ret < 0) {
				break;
			}

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_ASYNC_DMA
			ret = k_sem_take(&dma_read->status_sem,
					K_MSEC(CONFIG_MICRON_MT29FX_MEMC_NAND_ASYNC_DMA_TIMEOUT_MS));
			if (ret < 0) {
				break;
			}
#else
			for (;;) {
				struct dma_status stat;

				ret = dma_get_status(dma_read->dma_dev, dma_read->dma_channel, &stat);
				if (ret == 0 && stat.busy == false) {
					break;
				}
				if (ret < 0) {
					LOG_ERR("Error reading dma status [%d]", ret);
					break;
				}
			}
#endif

			data += dma_len;
			len -= dma_len;
		}
	}
#endif

	for (int i = 0; i < len; i++) {
		data[i] = *addr;
	}
}

static void flash_mt29fx_enable_ecc(const struct device *dev)
{
	uint8_t addr = NAND_FLASH_FEATURE_REG_ARRAY_OPERATIONS;
	struct flash_mt29fx_data *dev_data = dev->data;
	const struct flash_mt29fx_config *dev_config = dev->config;
	uint32_t array_operations;

	flash_mt29fx_wait_until_ready(dev_config);

	flash_mt29fx_write_command(dev_config, NAND_FLASH_SET_FEATURE_COMMAND);
	flash_mt29fx_write_address(dev_config, &addr, 1);

	// Wait at least for tADL (70ns) before sending data to the Flash
	// That's equivalent to ~10 instruction cycles running at max frequency (144MHz)
	// The line below will consume many more cycles
	for (int i = 0; i < 10; i++) {
		arch_nop();
	}

	array_operations = NAND_FLASH_FEATURE_REG_ARRAY_OPERATIONS_ECC_ENABLE;
	array_operations = sys_cpu_to_le32(array_operations);

	flash_mt29fx_write_data(dev_config, (uint8_t*)&array_operations, sizeof(array_operations));

	/* check if internal ecc is enabled */
	flash_mt29fx_write_command(dev_config, NAND_FLASH_GET_FEATURE_COMMAND);
	flash_mt29fx_write_address(dev_config, &addr, 1);

	for (int i = 0; i < 10; i++) {
		arch_nop();
	}

	flash_mt29fx_read_data(dev, (uint8_t*)&array_operations, sizeof(array_operations));
	array_operations = sys_le32_to_cpu(array_operations);
	if ((array_operations & NAND_FLASH_FEATURE_REG_ARRAY_OPERATIONS_ECC_ENABLE) == 0) {
		dev_data->internal_ecc_enabled = false;
	}
}

static uint8_t flash_mt29fx_read_status(const struct device *dev)
{
	const struct flash_mt29fx_config *dev_config = dev->config;
	uint8_t status;

	flash_mt29fx_write_command(dev_config, NAND_FLASH_GET_STATUS_COMMAND);
	flash_mt29fx_read_data(dev, &status, 1);

	return status;
}

static int flash_mt29fx_init(const struct device *dev)
{
	struct flash_mt29fx_data *dev_data = dev->data;
	const struct flash_mt29fx_config *dev_config = dev->config;
	k_sem_init(&dev_data->sem, 1, 1);

	if (dev_config->ready_busy_dt.port != NULL) {
		int ret;

		if (!device_is_ready(dev_config->ready_busy_dt.port)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&dev_config->ready_busy_dt, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}
	}

	if (dev_config->write_protect_dt.port != NULL) {
		int ret;

		if (!device_is_ready(dev_config->write_protect_dt.port)) {
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&dev_config->write_protect_dt,
					    GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);
		if (ret < 0) {
			return ret;
		}
	}


#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_DMA
	dma_init(dev);
#endif

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
	/* check if we have enough space in oob to store the ecc bytes */
	/* int partial_pages_per_page = dev_config->partial_partial__size; */
	if (dev_config->write_page_size % dev_config->ecc_chunk_size > 0) {
		LOG_ERR("Invalid ECC chunk size");
		return -EINVAL;
	}

	int ecc_overhead = dev_config->write_page_size / dev_config->ecc_chunk_size * ECC_OVERHEAD_PER_CHUNK;
	if (BAD_MARK_OVERHEAD + ecc_overhead > dev_config->oob_size) {
		return -EINVAL;
	}
#endif
	flash_mt29fx_write_command(dev_config, NAND_FLASH_RESET_COMMAND);
	flash_mt29fx_wait_until_ready(dev_config);
	flash_mt29fx_enable_ecc(dev);

	return 0;
}

static void flash_mt29fx_load_page_into_cache_reg(const struct flash_mt29fx_config *dev_config, off_t offset)
{
	uint64_t address_segment;
	uint32_t offset_page_start = ROUND_DOWN((uint32_t)offset, dev_config->write_page_size);

	flash_mt29fx_generate_address_segment(dev_config, offset_page_start, &address_segment, false);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_READ_PAGE_START_COMMAND);
	flash_mt29fx_write_address(dev_config, (uint8_t *)&address_segment, 5);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_READ_PAGE_STOP_COMMAND);
	flash_mt29fx_wait_until_ready(dev_config);
}

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
static void flash_mt29fx_read_change_column_address(const struct flash_mt29fx_config *dev_config, uint16_t column_address)
{
	column_address = sys_cpu_to_le16(column_address);

	flash_mt29fx_write_command(dev_config, NAND_FLASH_RANDOM_DATA_READ_START_COMMAND);
	flash_mt29fx_write_address(dev_config, (uint8_t *)&column_address, 2);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_RANDOM_DATA_READ_STOP_COMMAND);
}

static int flash_mt29fx_read_ecc_chunk(const struct device *dev,
				       int chunk_index, int chunk_offset, uint8_t *dst, size_t len)
{
	struct flash_mt29fx_data *dev_data = dev->data;
	const struct flash_mt29fx_config *dev_config = dev->config;
	uint16_t address_segment_chunk = chunk_index * dev_config->ecc_chunk_size;
	uint8_t ecc_bytes[ECC_OVERHEAD_PER_CHUNK];
	int ret;

	if (chunk_offset + len > dev_config->ecc_chunk_size) {
		return -EINVAL;
	}

	flash_mt29fx_read_change_column_address(dev_config, address_segment_chunk);

	/* read the data chunk */
	flash_mt29fx_read_data(dev, dev_data->internal_buffer, dev_config->ecc_chunk_size);

	/* change the column address to point to ecc bytes*/
	flash_mt29fx_generate_ecc_column_address(dev_config, chunk_index, &address_segment_chunk);
	flash_mt29fx_read_change_column_address(dev_config, address_segment_chunk);

	flash_mt29fx_read_data(dev, ecc_bytes, sizeof(ecc_bytes));

	ret = bch_verify(dev_config->bch_config, dev_data->internal_buffer, dev_config->ecc_chunk_size, ecc_bytes);

	if (ret) {
		LOG_ERR("Error reading chunk index %d, chunk offset %d", chunk_index, chunk_offset);
		bch_repair(dev_config->bch_config, dev_data->internal_buffer, dev_config->ecc_chunk_size,
			   ecc_bytes);
		ret = bch_verify(dev_config->bch_config, dev_data->internal_buffer, dev_config->ecc_chunk_size,
				 ecc_bytes);
	}

	if (ret < 0) {
		return ret;
	}

	memcpy(dst, dev_data->internal_buffer + chunk_offset, len);

	return 0;

}
#endif

/* read from a page at offset of size len */
/* if an external ecc engine is used, then we have to split up the read into */
/* ecc chunks, otherwise we can read the whole page. do we need to check the status register */
/* when this is done? */

static int flash_mt29fx_read_single_page(const struct device *dev, off_t offset, uint8_t *dst,
					 size_t len)
{
	const struct flash_mt29fx_config *dev_config = dev->config;;
	struct flash_mt29fx_data *dev_data = dev->data;;

	flash_mt29fx_load_page_into_cache_reg(dev_config, offset);

	/* when internal ecc is enabled, the status register must be checked if */
	/* the page was read without errors */
	if (dev_data->internal_ecc_enabled) {
		uint8_t status = flash_mt29fx_read_status(dev);

		if ((status & NAND_FLASH_STATUS_REWRITE_RECOMMENDED) != 0) {
			LOG_WRN("Re-write at offset 0x%x recommended", (uint32_t)offset);
		}
		if ((status & NAND_FLASH_STATUS_REG_FAIL) != 0) {
			LOG_ERR("Read failed due to uncorrectable ECC error");
			return -EINVAL;
		}

		/* simply read the data into the output buffer */
		flash_mt29fx_read_data(dev, dst, len);

		return 0;
	}

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
	/* how much do need to read from the first ecc chunk */
	int dst_offset = 0, read_size;
	uint16_t chunk_offset;
	uint32_t offset_page_start = ROUND_DOWN((uint32_t)offset, dev_config->write_page_size);
	int ecc_chunk_start_index = (offset - offset_page_start) / dev_config->ecc_chunk_size;
	int ret;

	read_size = MIN(ROUND_UP(offset, dev_config->ecc_chunk_size) - offset, len);
	chunk_offset = offset - ROUND_DOWN(offset, dev_config->ecc_chunk_size);

	if (read_size > 0) {
		ret = flash_mt29fx_read_ecc_chunk(dev, ecc_chunk_start_index, chunk_offset, &dst[dst_offset], read_size);
		if (ret < 0) {
			return ret;
		}

		dst_offset += read_size;
		len -= read_size;
		ecc_chunk_start_index++;
	}

	int num_pages = len / dev_config->ecc_chunk_size;
	read_size = dev_config->ecc_chunk_size;
	for (int i = 0; i < num_pages; i++) {
		ret = flash_mt29fx_read_ecc_chunk(dev, ecc_chunk_start_index, 0, &dst[dst_offset], read_size);
		if (ret < 0) {
			return ret;
		}

		dst_offset += read_size;
		len -= read_size;
		ecc_chunk_start_index++;
	}

	read_size = len;
	if (read_size > 0) {
		ret = flash_mt29fx_read_ecc_chunk(dev, ecc_chunk_start_index, 0, &dst[dst_offset], read_size);
		if (ret < 0) {
			return ret;
		}

		dst_offset += read_size;
		len -= read_size;
		ecc_chunk_start_index++;
	}
#else
	/* simply read the data into the output buffer */
	flash_mt29fx_read_data(dev, dst, len);
#endif
	return 0;

}

static int flash_mt29fx_read(const struct device *dev, off_t offset, void *data, size_t len)
{
	struct flash_mt29fx_data *dev_data = dev->data;
	const struct flash_mt29fx_config *dev_config = dev->config;
	int page_size = dev_config->parameters.write_block_size;
	uint8_t *dst = data;
	int dst_offset = 0;
	int read_size;
	int num_pages;
	int ret;

	if (offset < 0 || offset + len > dev_config->size) {
		return -1;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);

	read_size = MIN(ROUND_UP(offset, page_size) - offset, len);
	if (read_size > 0) {
		ret = flash_mt29fx_read_single_page(dev, offset, &dst[dst_offset], read_size);
		if (ret < 0) {
			LOG_ERR("Error reading page [%d]", ret);
			k_sem_give(&dev_data->sem);
			return ret;
		}
		dst_offset += read_size;
		offset += read_size;
		len -= read_size;
	}

	num_pages = len / page_size;
	read_size = page_size;
	for (int i = 0; i < num_pages; i++) {
		ret = flash_mt29fx_read_single_page(dev, offset, &dst[dst_offset], read_size);
		if (ret < 0) {
			LOG_ERR("Error reading page [%d]", ret);
			k_sem_give(&dev_data->sem);
			return ret;
		}
		dst_offset += read_size;
		offset += read_size;
		len -= read_size;
	}

	read_size = len;
	if (read_size > 0) {
		ret = flash_mt29fx_read_single_page(dev, offset, &dst[dst_offset], read_size);
		if (ret < 0) {
			LOG_ERR("Error reading page [%d]", ret);
			k_sem_give(&dev_data->sem);
			return ret;
		}
		dst_offset += read_size;
		offset += read_size;
		len -= read_size;
	}

	k_sem_give(&dev_data->sem);

	return 0;
}

static int flash_mt29fx_write_single_partial_page(const struct device *dev, off_t offset,
						  const uint8_t *src, size_t len)
{
	int ret = 0;
	uint8_t status;
	uint64_t address_segment;
	const struct flash_mt29fx_config *dev_config = dev->config;

	flash_mt29fx_generate_address_segment(dev_config, offset, &address_segment, false);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_PROGRAM_PAGE_START_COMMAND);
	flash_mt29fx_write_address(dev_config, (uint8_t *)&address_segment, 5);
	flash_mt29fx_write_data(dev_config, src, len);

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
	struct flash_mt29fx_data *dev_data = dev->data;
	if (!dev_data->internal_ecc_enabled) {
		uint32_t offset_page_start = ROUND_DOWN((uint32_t)offset, dev_config->write_page_size);
		int ecc_chunk_size = dev_config->ecc_chunk_size;
		int ecc_chunk_start_index = (offset - offset_page_start) / ecc_chunk_size;
		int ecc_chunks = len / ecc_chunk_size;

		for (int i = 0; i < ecc_chunks; i++) {
			uint8_t ecc_bytes[ECC_OVERHEAD_PER_CHUNK];
			uint16_t address_segment_ecc;

			bch_generate(dev_config->bch_config, &src[i * dev_config->ecc_chunk_size],
				     ecc_chunk_size, ecc_bytes);

			flash_mt29fx_generate_ecc_column_address(
				dev_config, ecc_chunk_start_index + i, &address_segment_ecc);
			address_segment_ecc = sys_cpu_to_le16(address_segment_ecc);

			flash_mt29fx_write_command(dev_config,
						   NAND_FLASH_CHANGE_WRITE_COLUMN_COMMAND);
			flash_mt29fx_write_address(dev_config, (uint8_t *)&address_segment_ecc, 2);
			flash_mt29fx_write_data(dev_config, ecc_bytes, sizeof(ecc_bytes));
		}
	}
#endif

	flash_mt29fx_write_command(dev_config, NAND_FLASH_PROGRAM_PAGE_STOP_COMMAND);
	flash_mt29fx_wait_until_ready(dev_config);

	status = flash_mt29fx_read_status(dev);

	if (status & NAND_FLASH_STATUS_REG_FAIL) {
		ret = -EINVAL;
	}

	return ret;
}

static int flash_mt29fx_write(const struct device *dev, off_t offset, const void *data, size_t len)
{
	struct flash_mt29fx_data *dev_data = dev->data;
	const struct flash_mt29fx_config *dev_config = dev->config;
	int partial_page_size = dev_config->partial_write_page_size;
	const uint8_t *src = data;
	int src_offset = 0;
	int num_pages;
	int program_size;
	int ret = 0, ret_tmp;

	if (offset < 0 || offset + len > dev_config->size) {
		return -EINVAL;
	}

	if (offset % partial_page_size > 0 || (offset + len) % partial_page_size > 0) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);

	/* program the first page upto the program_page_size boundary */
	num_pages = len / partial_page_size;
	program_size = partial_page_size;
	for (int i = 0; i < num_pages; i++) {
		ret_tmp = flash_mt29fx_write_single_partial_page(dev, offset, &src[src_offset],
								 program_size);
		if (ret_tmp < 0) {
			ret = ret_tmp;
		}
		src_offset += program_size;
		offset += program_size;
		len -= program_size;
	}

	k_sem_give(&dev_data->sem);

	return ret;
}

static int flash_mt29fx_is_page_erased(const struct device *dev, off_t offset, int page_size, bool *is_erased)
{
	const struct flash_mt29fx_config *dev_config = dev->config;
	struct flash_mt29fx_data *dev_data = dev->data;
	uint64_t address_segment;
	*is_erased = true;

	if (offset < 0 || offset > dev_config->size || offset % page_size > 0) {
		return -EINVAL;
	}

	k_sem_take(&dev_data->sem, K_FOREVER);

	flash_mt29fx_generate_address_segment(dev_config, offset, &address_segment, false);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_READ_PAGE_START_COMMAND);
	flash_mt29fx_write_address(dev_config, (uint8_t *)&address_segment, 5);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_READ_PAGE_STOP_COMMAND);
	flash_mt29fx_wait_until_ready(dev_config);

	for (int i = 0; i < page_size; i++) {
		uint8_t data;

		flash_mt29fx_read_data(dev, &data, 1);
		if (data != ERASE_VALUE) {
			*is_erased = false;
			break;
		}
	}

	k_sem_give(&dev_data->sem);
	return 0;
}

static int flash_mt29fx_is_bad_block(const struct device *dev, uint64_t block_index, bool *is_bad_block)
{
	const struct flash_mt29fx_config *dev_config = dev->config;
	struct flash_mt29fx_data *dev_data = dev->data;
	int block_size = dev_config->layout.pages_size;
	int page_size = dev_config->parameters.write_block_size;
	uint64_t address_segment, offset;
	uint8_t bad_block_mark;

	if (block_index >= dev_config->num_blocks) {
		return -EINVAL;
	}

	offset = block_index * block_size;

	k_sem_take(&dev_data->sem, K_FOREVER);

	/* check the first block */
	flash_mt29fx_generate_address_segment(dev_config, offset, &address_segment, true);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_READ_PAGE_START_COMMAND);
	flash_mt29fx_write_address(dev_config, (uint8_t *)&address_segment, 5);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_READ_PAGE_STOP_COMMAND);
	flash_mt29fx_wait_until_ready(dev_config);
	flash_mt29fx_read_data(dev, &bad_block_mark, 1);

	if (bad_block_mark == 0) {
		*is_bad_block = true;
		k_sem_give(&dev_data->sem);
		return 0;
	}

	/* check the last block */
	offset += block_size - page_size;

	flash_mt29fx_generate_address_segment(dev_config, offset, &address_segment, true);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_READ_PAGE_START_COMMAND);
	flash_mt29fx_write_address(dev_config, (uint8_t *)&address_segment, 5);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_READ_PAGE_STOP_COMMAND);
	flash_mt29fx_wait_until_ready(dev_config);
	flash_mt29fx_read_data(dev, &bad_block_mark, 1);

	if (bad_block_mark == 0) {
		*is_bad_block = true;
	} else {
		*is_bad_block = false;
	}

	k_sem_give(&dev_data->sem);
	return 0;
}


static int flash_mt29fx_erase(const struct device *dev, off_t offset, size_t size)
{
	struct flash_mt29fx_data *dev_data = dev->data;
	const struct flash_mt29fx_config *dev_config = dev->config;
	uint64_t address_segment;
	int block_size = dev_config->layout.pages_size;
	int num_blocks = size / block_size;
	int ret_tmp, ret = 0;

	if (offset < 0 || offset + size > dev_config->size || offset % block_size > 0 ||
	    size % block_size > 0) {
		return -EINVAL;
	}

	for (int i = 0; i < num_blocks; i++) {
		bool is_bad_block;
		uint8_t status;

		ret_tmp = flash_mt29fx_is_bad_block(dev, i + offset / block_size, &is_bad_block);
		if (ret_tmp < 0) {
			ret = ret_tmp;
			continue;
		}

		if (is_bad_block) {
			LOG_WRN("Bad block %d - skipping erase", (uint32_t)(i + offset / block_size));
			continue;
		}

		flash_mt29fx_generate_address_segment(dev_config, offset + i * block_size, &address_segment, false);
		k_sem_take(&dev_data->sem, K_FOREVER);
		flash_mt29fx_write_command(dev_config, NAND_FLASH_ERASE_BLOCK_START_COMMAND);
		flash_mt29fx_write_address(dev_config, (uint8_t *)&address_segment + 2, 3);
		flash_mt29fx_write_command(dev_config, NAND_FLASH_ERASE_BLOCK_STOP_COMMAND);
		flash_mt29fx_wait_until_ready(dev_config);

		status = flash_mt29fx_read_status(dev);
		if (status & NAND_FLASH_STATUS_REG_FAIL) {
			LOG_ERR("Erase operation at Block Index %d failed", (uint32_t)(i + offset / block_size));
			ret = -EINVAL;
		}

		k_sem_give(&dev_data->sem);
	}
	return ret;
}

static const struct flash_parameters *flash_mt29fx_get_parameters(const struct device *dev)
{
	const struct flash_mt29fx_config *dev_config = dev->config;

	return &dev_config->parameters;
}


static int flash_mt29fx_mark_bad_block(const struct device *dev, off_t offset)
{
	struct flash_mt29fx_data *dev_data = dev->data;
	const struct flash_mt29fx_config *dev_config = dev->config;
	int block_size = dev_config->layout.pages_size;
	uint64_t address_segment;
	uint8_t bad_block_mark = 0;

	if (offset < 0 || offset >= dev_config->size || offset % block_size > 0) {
		return -EINVAL;
	}

	LOG_ERR("marking block 0x%x bad", (uint32_t)offset);

	k_sem_take(&dev_data->sem, K_FOREVER);

	flash_mt29fx_generate_address_segment(dev_config, offset, &address_segment, true);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_PROGRAM_PAGE_START_COMMAND);
	flash_mt29fx_write_address(dev_config, (uint8_t *)&address_segment, 5);
	flash_mt29fx_write_data(dev_config, &bad_block_mark, 1);
	flash_mt29fx_write_command(dev_config, NAND_FLASH_PROGRAM_PAGE_STOP_COMMAND);
	flash_mt29fx_wait_until_ready(dev_config);

	k_sem_give(&dev_data->sem);

	return 0;
}
static int flash_mt29fx_ex_op(const struct device *dev, uint16_t code, const uintptr_t in,
			      void *out)
{
	const struct flash_mt29fx_config *dev_config = dev->config;

	if (code == FLASH_EX_OP_IS_BAD_BLOCK) {
		return flash_mt29fx_is_bad_block(dev, in, out);
	}

	if (code == FLASH_EX_OP_MARK_BAD_BLOCK) {
		return flash_mt29fx_mark_bad_block(dev, in);
	}

	if (code == FLASH_EX_OP_IS_PAGE_ERASED) {
		return flash_mt29fx_is_page_erased(dev, in, dev_config->write_page_size, out);
	}

	if (code == FLASH_EX_OP_IS_PARTIAL_PAGE_ERASED) {
		return flash_mt29fx_is_page_erased(dev, in, dev_config->partial_write_page_size, out);
	}

	if (code == FLASH_EX_OP_GET_PARTIAL_PAGE_SIZE) {
		*(int*)out = dev_config->partial_write_page_size;
		return 0;
	}

	return -ENOTSUP;
}

static const struct flash_driver_api flash_mt29fx_api = {
	.erase = flash_mt29fx_erase,
	.write = flash_mt29fx_write,
	.read = flash_mt29fx_read,
#ifdef CONFIG_FLASH_PAGE_LAYOUT
	.page_layout = flash_mt29fx_get_page_layout,
#endif
	.get_parameters = flash_mt29fx_get_parameters,
	.ex_op = flash_mt29fx_ex_op,
};

#define BLOCK_SIZE(inst)                                                                           \
	(DT_INST_PROP(inst, write_page_size) * DT_INST_PROP(inst, pages_per_block))


#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
#define EXTERNAL_ECC_DECLARE(inst)                                                                 \
	.ecc_chunk_size = DT_INST_PROP(inst, ecc_chunk_size_bytes),                                \
	.oob_size = DT_INST_PROP(inst, oob_size_bytes),						   \
	.bch_config = &bch_4bit,
#else
#define EXTERNAL_ECC_DECLARE(inst)
#endif

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
#define INTERNAL_BUFFER_DECLARE(inst)                                                              \
	.internal_buffer = flash_mt29fx_internal_buffer_##inst,                                    \
	.internal_buffer_size = DT_INST_PROP(inst, partial_write_page_size),
#else
#define INTERNAL_BUFFER_DECLARE(inst)
#endif

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_EXTERNAL_ECC
#define INTERNAL_BUFFER_DEFINE(inst)                                                               \
	uint8_t flash_mt29fx_internal_buffer_##inst[DT_INST_PROP(inst, partial_write_page_size)];
#else
#define INTERNAL_BUFFER_DEFINE(inst)
#endif

#ifdef CONFIG_MICRON_MT29FX_MEMC_NAND_USE_DMA
#define DMA_CHANNEL_INIT(index, dir, src_burst, dst_burst)                                         \
	.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_NAME(index, dir)),                           \
	.dma_channel = DT_INST_DMAS_CELL_BY_NAME(index, dir, channel),                             \
	.dma_cfg = {                                                                               \
		.channel_direction = MEMORY_TO_MEMORY,                                             \
		.channel_priority = DT_INST_DMAS_CELL_BY_NAME(index, dir, priority),               \
		.source_data_size = 1,                                                             \
		.dest_data_size = 1,                                                               \
		.source_burst_length = src_burst,                                                  \
		.dest_burst_length = dst_burst,                                                    \
		.block_count = 1,                                                                  \
		.dma_callback = dma_##dir##_cb,                                                    \
	},

#define DMA_CHANNEL(index, dir, src_burst, dst_burst)                                              \
	.dma_##dir = {COND_CODE_1(                                                                 \
		DT_INST_DMAS_HAS_NAME(index, dir),                                                 \
		(DMA_CHANNEL_INIT(index, dir, src_burst, dst_burst)), (NULL))},
#else
#define DMA_CHANNEL(index, dir, src_burst, dst_burst)
#endif

#define FLASH_MT29FX_INIT(inst)                                                                    \
	INTERNAL_BUFFER_DEFINE(inst)								   \
	static struct flash_mt29fx_data flash_mt29fx_data_##inst = {                               \
		INTERNAL_BUFFER_DECLARE(inst)							   \
		DMA_CHANNEL(inst, write, 8, 8)							   \
		DMA_CHANNEL(inst, read, 8, 8)							   \
 	};                                                                                         \
	static struct flash_mt29fx_config flash_mt29fx_cfg_##inst = {                              \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.size = (uint64_t)DT_INST_PROP(inst, num_blocks) * BLOCK_SIZE(inst),               \
		.num_blocks = DT_INST_PROP(inst, num_blocks),                                      \
		.ale_mask = DT_INST_PROP(inst, ale_address_mask),                                  \
		.cle_mask = DT_INST_PROP(inst, cle_address_mask),                                  \
		.parameters = {.write_block_size = DT_INST_PROP(inst, write_page_size),            \
			       .erase_value = ERASE_VALUE},                                        \
		.layout = {.pages_size = BLOCK_SIZE(inst),                                         \
			   .pages_count = DT_INST_PROP(inst, num_blocks) },                        \
		.ready_busy_dt = GPIO_DT_SPEC_INST_GET_OR(inst, ready_busy_gpios, {0}),            \
		.write_protect_dt = GPIO_DT_SPEC_INST_GET_OR(inst, write_protect_gpios, {0}),      \
		.write_page_size = DT_INST_PROP(inst, write_page_size),				   \
		.block_size = BLOCK_SIZE(inst),                                                    \
		.partial_write_page_size = DT_INST_PROP(inst, partial_write_page_size),            \
		.memc_dev = DEVICE_DT_GET(DT_INST_PROP(inst, memc_handle)),                        \
		.memc_index = DT_INST_PROP(inst, memc_index),                                      \
		EXTERNAL_ECC_DECLARE(inst)							   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, flash_mt29fx_init, NULL, &flash_mt29fx_data_##inst,            \
			      &flash_mt29fx_cfg_##inst, POST_KERNEL, CONFIG_FLASH_INIT_PRIORITY,   \
			      &flash_mt29fx_api);

DT_INST_FOREACH_STATUS_OKAY(FLASH_MT29FX_INIT)
