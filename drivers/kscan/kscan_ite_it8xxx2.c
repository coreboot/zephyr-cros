/*
 * Copyright (c) 2021 ITE Corporation. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ite_it8xxx2_kscan

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/kscan.h>
#include <drivers/pinmux.h>
#include <errno.h>
#include <kernel.h>
#include <soc.h>
#include <soc_dt.h>
#include <sys/atomic.h>

#include <logging/log.h>
#define LOG_LEVEL CONFIG_KSCAN_LOG_LEVEL
LOG_MODULE_REGISTER(kscan_ite_it8xxx2);

#define KEYBOARD_COLUMN_DRIVE_ALL	-2
#define KEYBOARD_COLUMN_DRIVE_NONE	-1
/* Free run timer counts transform to micro-seconds (clock source is 32768Hz) */
#define CLOCK_32K_HW_CYCLES_TO_US(X)	\
	(uint32_t)((((uint64_t)(X) * 1000000U) /  \
		     sys_clock_hw_cycles_per_sec()))
/* Milli-second transform to micro-second */
#define MS_TO_US			1000U
/* Number of tracked scan times */
#define SCAN_OCURRENCES			30U
/* Thread stack size */
#define TASK_STACK_SIZE			1024

/* Device config */
enum kscan_pin_func {
	KSO16 = 0,
	KSO17,
};

struct kscan_alt_cfg {
	/* Pinmux control device structure */
	const struct device *pinctrls;
	/* GPIO pin */
	uint8_t pin;
	/* Alternate function */
	uint8_t alt_fun;
};

struct kscan_it8xxx2_config {
	/* Keyboard scan controller base address */
	uintptr_t base;
	/* KSI[7:0] wake-up edge mode register */
	uintptr_t reg_wuemr3;
	/* KSI[7:0] wake-up edge sense register */
	uintptr_t reg_wuesr3;
	/* KSI[7:0] wake-up enable register */
	uintptr_t reg_wuenr3;
	/* Keyboard scan input (KSI) wake-up irq */
	int irq;
	/* GPIO control device structure */
	const struct device *gpio_dev;
	/* Keyboard scan alternate configuration list */
	const struct kscan_alt_cfg *alt_list;
};

/* Device data */
struct kscan_it8xxx2_data {
	/* Variables in usec units */
	uint32_t deb_time_press;
	uint32_t deb_time_rel;
	int32_t poll_timeout;
	uint32_t poll_period;
	uint8_t matrix_stable_state[CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE];
	uint8_t matrix_unstable_state[CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE];
	uint8_t matrix_previous_state[CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE];
	/* Index in to the scan_clock_cycle to indicate start of debouncing */
	uint8_t scan_cycle_idx[CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE]
			      [CONFIG_KSCAN_ITE_IT8XXX2_ROW_SIZE];
	/*
	 * Track previous "elapsed clock cycles" per matrix scan. This
	 * is used to calculate the debouncing time for every key
	 */
	uint8_t scan_clk_cycle[SCAN_OCURRENCES];
	struct k_sem poll_lock;
	uint8_t scan_cycles_idx;
	kscan_callback_t callback;
	struct k_thread thread;
	atomic_t enable_scan;

	K_KERNEL_STACK_MEMBER(thread_stack, TASK_STACK_SIZE);
};

/* Driver convenience defines */
#define DRV_CONFIG(dev)	((const struct kscan_it8xxx2_config *)(dev)->config)
#define DRV_DATA(dev)	((struct kscan_it8xxx2_data *)(dev)->data)
#define DRV_REG(dev)	(struct kscan_it8xxx2_regs *)(DRV_CONFIG(dev)->base)

static void drive_keyboard_column(const struct device *dev, int col)
{
	struct kscan_it8xxx2_regs *const inst = DRV_REG(dev);
	int mask;

	/* Tri-state all outputs */
	if (col == KEYBOARD_COLUMN_DRIVE_NONE)
		mask = 0x3ffff;
	/* Assert all outputs */
	else if (col == KEYBOARD_COLUMN_DRIVE_ALL)
		mask = 0;
	/* Assert a single output */
	else
		mask = 0x3ffff ^ BIT(col);

	/* Set KSO[17:0] output data */
	inst->KBS_KSOL = (uint8_t) (mask & 0xff);
	inst->KBS_KSOH1 = (uint8_t) ((mask >> 8) & 0xff);
#if (CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE > 16)
	inst->KBS_KSOH2 = (uint8_t) ((mask >> 16) & 0xff);
#endif
}

static uint8_t read_keyboard_row(const struct device *dev)
{
	struct kscan_it8xxx2_regs *const inst = DRV_REG(dev);

	/* Bits are active-low, so toggle it (return 1 means key pressed) */
	return (inst->KBS_KSI ^ 0xff);
}

static bool is_matrix_ghosting(const uint8_t *state)
{
	/*
	 * Matrix keyboard designs are suceptible to ghosting.
	 * An extra key appears to be pressed when 3 keys
	 * belonging to the same block are pressed.
	 * for example, in the following block
	 *
	 * . . w . q .
	 * . . . . . .
	 * . . . . . .
	 * . . m . a .
	 *
	 * the key m would look as pressed if the user pressed keys
	 * w, q and a simultaneously. A block can also be formed,
	 * with not adjacent columns.
	 */
	for (int c = 0; c < CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE; c++) {
		if (!state[c])
			continue;

		for (int c_n = c + 1; c_n < CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE; c_n++) {
			/*
			 * We AND the columns to detect a "block".
			 * this is an indication of ghosting, due to current
			 * flowing from a key which was never pressed. in our
			 * case, current flowing is a bit set to 1 as we
			 * flipped the bits when the matrix was scanned.
			 * now we OR the columns using z&(z-1) which is
			 * non-zero only if z has more than one bit set.
			 */
			uint8_t common_row_bits = state[c] & state[c_n];

			if (common_row_bits & (common_row_bits - 1))
				return true;
		}
	}

	return false;
}

static bool read_keyboard_matrix(const struct device *dev, uint8_t *new_state)
{
	uint8_t row;
	uint8_t key_event = 0U;

	for (int col = 0; col < CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE; col++) {
		/* Drive specific column low and others high */
		drive_keyboard_column(dev, col);
		/* Allow the matrix to stabilize before reading it */
		k_busy_wait(50U);
		row = read_keyboard_row(dev);
		new_state[col] = row;

		key_event |= row;
	}

	drive_keyboard_column(dev, KEYBOARD_COLUMN_DRIVE_NONE);

	return key_event != 0U ? true : false;
}

static void keyboard_raw_interrupt(const struct device *dev)
{
	const struct kscan_it8xxx2_config *const config = DRV_CONFIG(dev);
	volatile uint8_t *reg_wuesr3 = (uint8_t *)config->reg_wuesr3;
	struct kscan_it8xxx2_data *data = DRV_DATA(dev);

	/* W/C wakeup interrupt status of KSI[0:7] pins */
	*reg_wuesr3 = 0xFF;

	/* W/C interrupt status of KSI[7:0] pins */
	ite_intc_isr_clear(config->irq);

	/* Release poll lock semaphore */
	k_sem_give(&data->poll_lock);
}

void keyboard_raw_enable_interrupt(const struct device *dev, int enable)
{
	const struct kscan_it8xxx2_config *const config = DRV_CONFIG(dev);
	volatile uint8_t *reg_wuesr3 = (uint8_t *)config->reg_wuesr3;

	if (enable) {
		/* W/C wakeup interrupt status of KSI[0:7] pins */
		*reg_wuesr3 = 0xFF;

		/* W/C interrupt status of KSI[7:0] pins */
		ite_intc_isr_clear(config->irq);

		irq_enable(config->irq);
	} else {
		irq_disable(config->irq);
	}
}

static bool check_key_events(const struct device *dev)
{
	struct kscan_it8xxx2_data *data = DRV_DATA(dev);
	uint8_t matrix_new_state[CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE] = {0U};
	bool key_pressed = false;
	uint32_t cycles_now  = k_cycle_get_32();
	uint8_t row_changed = 0U;
	uint8_t deb_col;

	if (++data->scan_cycles_idx >= SCAN_OCURRENCES)
		data->scan_cycles_idx = 0U;

	data->scan_clk_cycle[data->scan_cycles_idx] = cycles_now;

	/* Scan the matrix */
	key_pressed = read_keyboard_matrix(dev, matrix_new_state);

	/* Abort if ghosting is detected */
	if (is_matrix_ghosting(matrix_new_state)) {
		return false;
	}

	/*
	 * The intent of this loop is to gather information related to key
	 * changes
	 */
	for (int c = 0; c < CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE; c++) {
		/* Check if there was an update from the previous scan */
		row_changed = matrix_new_state[c] ^
			      data->matrix_previous_state[c];

		if (!row_changed)
			continue;

		for (int r = 0; r < CONFIG_KSCAN_ITE_IT8XXX2_ROW_SIZE; r++) {
			/*
			 * Index all they keys that changed for each row
			 * in order to debounce each key in terms of it
			 */
			if (row_changed & BIT(r))
				data->scan_cycle_idx[c][r] =
					data->scan_cycles_idx;
		}

		data->matrix_unstable_state[c] |= row_changed;
		data->matrix_previous_state[c] = matrix_new_state[c];
	}

	for (int c = 0; c < CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE; c++) {
		deb_col = data->matrix_unstable_state[c];

		if (!deb_col)
			continue;

		/* Debouncing for each row key occurs here */
		for (int r = 0; r < CONFIG_KSCAN_ITE_IT8XXX2_ROW_SIZE; r++) {
			uint8_t mask = BIT(r);
			uint8_t row_bit = matrix_new_state[c] & mask;

			/* Continue if we already debounce a key */
			if (!(deb_col & mask))
				continue;

			/* Convert the clock cycle differences to usec */
			uint32_t debt = CLOCK_32K_HW_CYCLES_TO_US(cycles_now -
			data->scan_clk_cycle[data->scan_cycle_idx[c][r]]);

			/* Does the key requires more time to be debounced ? */
			if (debt < (row_bit ? data->deb_time_press :
				    data->deb_time_rel)) {
				/* Need more time to debounce */
				continue;
			}

			data->matrix_unstable_state[c] &= ~row_bit;

			/* Check if there was a change in the stable state */
			if ((data->matrix_stable_state[c] & mask) == row_bit) {
				/* Key state did not change */
				continue;
			}

			/*
			 * The current row has been debounced, therefore update
			 * the stable state. Then, proceed to notify the
			 * application about the keys pressed.
			 */
			data->matrix_stable_state[c] ^= mask;
			if ((atomic_get(&data->enable_scan) == 1U) &&
			    (data->callback != NULL)) {
				data->callback(dev, r, c,
					       row_bit ? true : false);
			}
		}
	}

	return key_pressed;
}

/**
 * @brief Determine if a timer is expired.
 *
 * @param start_cycles The starting time of HW cycle.
 * @param timeout Pointer to the period time.
 *
 * @retval true If timer is expired;
 *         false If timer isn't expired.
 */
static bool poll_expired(uint32_t start_cycles, int32_t *timeout)
{
	uint32_t now_cycles;
	uint32_t microsecs_spent;

	now_cycles = k_cycle_get_32();
	microsecs_spent = CLOCK_32K_HW_CYCLES_TO_US(now_cycles - start_cycles);

	/* Update the timeout value */
	*timeout -= microsecs_spent;

	return !(*timeout >= 0);
}

void polling_task(const struct device *dev, void *dummy2, void *dummy3)
{
	struct kscan_it8xxx2_data *data = DRV_DATA(dev);
	int32_t local_poll_timeout = data->poll_timeout;
	uint32_t current_cycles;
	uint32_t cycles_delta;
	uint32_t wait_period;

	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	while (true) {
		/* Init all KSO output low */
		drive_keyboard_column(dev, KEYBOARD_COLUMN_DRIVE_ALL);
		/* Enable wakeup and interrupt of KSI pins */
		keyboard_raw_enable_interrupt(dev, 1);
		/* Wait poll lock semaphore */
		k_sem_take(&data->poll_lock, K_FOREVER);
		/* Disable wakeup and interrupt of KSI pins after fired */
		keyboard_raw_enable_interrupt(dev, 0);

		uint32_t start_poll_cycles = k_cycle_get_32();

		while (atomic_get(&data->enable_scan) == 1U) {
			uint32_t start_period_cycles = k_cycle_get_32();

			if (check_key_events(dev)) {
				start_poll_cycles = k_cycle_get_32();
			} else if (poll_expired(start_poll_cycles,
						&local_poll_timeout)) {
				break;
			}

			/*
			 * Subtract the time invested from the sleep period
			 * in order to compensate for the time invested
			 * in debouncing a key
			 */
			current_cycles = k_cycle_get_32();
			cycles_delta = current_cycles - start_period_cycles;
			wait_period = data->poll_period -
				      CLOCK_32K_HW_CYCLES_TO_US(cycles_delta);

			/* Override wait_period in case it's less than 1000 us */
			if (wait_period < MS_TO_US)
				wait_period = MS_TO_US;

			/*
			 * Wait period results in a larger number when
			 * current cycles counter wrap. In this case, the
			 * whole poll period is used
			 */
			if (wait_period > data->poll_period) {
				LOG_DBG("wait_period : %u", wait_period);
				wait_period = data->poll_period;
			}

			/* Allow other threads to run while we sleep */
			k_usleep(wait_period);
		}
	}
}

static int kscan_it8xxx2_init(const struct device *dev)
{
	const struct kscan_it8xxx2_config *const config = DRV_CONFIG(dev);
	volatile uint8_t *reg_wuemr3 = (uint8_t *)config->reg_wuemr3;
	volatile uint8_t *reg_wuesr3 = (uint8_t *)config->reg_wuesr3;
	volatile uint8_t *reg_wuenr3 = (uint8_t *)config->reg_wuenr3;
	struct kscan_it8xxx2_data *data = DRV_DATA(dev);
	struct kscan_it8xxx2_regs *const inst = DRV_REG(dev);

	/* Disable wakeup and interrupt of KSI pins before configuring */
	keyboard_raw_enable_interrupt(dev, 0);

	/*
	 * Bit[2] = 1: Enable the internal pull-up of KSO[15:0] pins
	 * Bit[0] = 1: Enable the open-drain mode of KSO[17:0] pins
	 */
	inst->KBS_KSOCTRL = (IT8XXX2_KBS_KSOOD | IT8XXX2_KBS_KSOPU);

#if (CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE > 16)
	/*
	 * For KSO[16] and KSO[17]:
	 * 1.GPOTRC:
	 *   Bit[x] = 1b: Enable the open-drain mode of KSO pin
	 * 2.GPCRCx:
	 *   Bit[7:6] = 00b: Select alternate KSO function
	 *   Bit[2] = 1b: Enable the internal pull-up of KSO pin
	 *
	 * NOTE: Set input temporarily for gpio_pin_configure(), after that
	 *       pinmux_pin_set() set to alternate function immediately.
	 */
	gpio_pin_configure(config->gpio_dev,
			   config->alt_list[KSO16].pin,
			   (GPIO_OPEN_DRAIN | GPIO_INPUT | GPIO_PULL_UP));
	gpio_pin_configure(config->gpio_dev,
			   config->alt_list[KSO17].pin,
			   (GPIO_OPEN_DRAIN | GPIO_INPUT | GPIO_PULL_UP));
	pinmux_pin_set(config->alt_list[KSO16].pinctrls,
		       config->alt_list[KSO16].pin,
		       config->alt_list[KSO16].alt_fun);
	pinmux_pin_set(config->alt_list[KSO17].pinctrls,
		       config->alt_list[KSO17].pin,
		       config->alt_list[KSO17].alt_fun);
#endif

	/* Bit[2] = 1: Enable the internal pull-up of KSI[7:0] pins */
	inst->KBS_KSICTRL = IT8XXX2_KBS_KSIPU;

	/* KSO[17:0] pins output low */
	inst->KBS_KSOL = 0x00;
	inst->KBS_KSOH1 = 0x00;
#if (CONFIG_KSCAN_ITE_IT8XXX2_COLUMN_SIZE > 16)
	inst->KBS_KSOH2 = 0x00;
#endif

	/* Select wakeup interrupt falling-edge triggered of KSI[7:0] pins */
	*reg_wuemr3 = 0xFF;

	/* W/C wakeup interrupt status of KSI[7:0] pins */
	*reg_wuesr3 = 0xFF;

	/* W/C interrupt status of KSI[7:0] pins */
	ite_intc_isr_clear(config->irq);

	/* Enable wakeup interrupt of KSI[7:0] pins */
	*reg_wuenr3 = 0xFF;

	/* Kconfig.it8xxx2 time figures are transformed from msec to usec */
	data->deb_time_press =
		(uint32_t) (CONFIG_KSCAN_ITE_IT8XXX2_DEBOUNCE_DOWN * MS_TO_US);
	data->deb_time_rel =
		(uint32_t) (CONFIG_KSCAN_ITE_IT8XXX2_DEBOUNCE_UP * MS_TO_US);
	data->poll_period =
		(uint32_t) (CONFIG_KSCAN_ITE_IT8XXX2_POLL_PERIOD * MS_TO_US);
	data->poll_timeout = 100 * MS_TO_US;

	/* Init null for callback function */
	data->callback = NULL;

	/* Create poll lock semaphore */
	k_sem_init(&data->poll_lock, 0, 1);

	/* Enable keyboard scan loop */
	atomic_set(&data->enable_scan, 1);

	/* Create keyboard scan task */
	k_thread_create(&data->thread, data->thread_stack,
			TASK_STACK_SIZE,
			(void (*)(void *, void *, void *))polling_task,
			(void *)dev, NULL, NULL,
			K_PRIO_COOP(4), 0, K_NO_WAIT);

	irq_connect_dynamic(DT_INST_IRQN(0), 0,
			    (void (*)(const void *))keyboard_raw_interrupt,
			    (const void *)dev, 0);

	return 0;
}

static int kscan_it8xxx2_configure(const struct device *dev,
					kscan_callback_t callback)
{
	struct kscan_it8xxx2_data *data = DRV_DATA(dev);

	if (!callback) {
		return -EINVAL;
	}

	/* Setup callback function */
	data->callback = callback;

	return 0;
}

static int kscan_it8xxx2_disable_callback(const struct device *dev)
{
	struct kscan_it8xxx2_data *data = DRV_DATA(dev);

	/* Disable keyboard scan loop */
	atomic_set(&data->enable_scan, 0);

	return 0;
}

static int kscan_it8xxx2_enable_callback(const struct device *dev)
{
	struct kscan_it8xxx2_data *data = DRV_DATA(dev);

	/* Enable keyboard scan loop */
	atomic_set(&data->enable_scan, 1);

	return 0;
}

static const struct kscan_driver_api kscan_it8xxx2_driver_api = {
	.config = kscan_it8xxx2_configure,
	.disable_callback = kscan_it8xxx2_disable_callback,
	.enable_callback = kscan_it8xxx2_enable_callback,
};

static const struct kscan_alt_cfg kscan_alt_0[DT_INST_NUM_PINCTRLS_BY_IDX(0, 0)] =
		IT8XXX2_DT_ALT_ITEMS_LIST(0);

static const struct kscan_it8xxx2_config kscan_it8xxx2_cfg_0 = {
	.base = DT_INST_REG_ADDR_BY_IDX(0, 0),
	.reg_wuemr3 = DT_INST_REG_ADDR_BY_IDX(0, 1),
	.reg_wuesr3 = DT_INST_REG_ADDR_BY_IDX(0, 2),
	.reg_wuenr3 = DT_INST_REG_ADDR_BY_IDX(0, 3),
	.irq = DT_INST_IRQN(0),
	.gpio_dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(DT_DRV_INST(0), gpio_dev, 0)),
	.alt_list = kscan_alt_0,
};

static struct kscan_it8xxx2_data kscan_it8xxx2_kbd_data;

DEVICE_DT_INST_DEFINE(0,
			&kscan_it8xxx2_init,
			NULL,
			&kscan_it8xxx2_kbd_data,
			&kscan_it8xxx2_cfg_0,
			POST_KERNEL,
			CONFIG_KSCAN_INIT_PRIORITY,
			&kscan_it8xxx2_driver_api);
