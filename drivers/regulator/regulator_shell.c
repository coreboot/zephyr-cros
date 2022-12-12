/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/regulator.h>

LOG_MODULE_REGISTER(regulator_shell, CONFIG_REGULATOR_LOG_LEVEL);

static int cmd_reg_en(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *reg_dev;
	int ret;

	reg_dev = device_get_binding(argv[1]);
	if (reg_dev == NULL) {
		shell_error(sh, "regulator device %s not available", argv[1]);
		return -ENODEV;
	}

	ret = regulator_enable(reg_dev);
	if (ret < 0) {
		shell_error(sh, "failed to enable regulator, error %d", ret);
		return ret;
	}
	shell_print(sh, "enabled regulator");
	return 0;
}


static int cmd_reg_dis(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *reg_dev;
	int ret;

	reg_dev = device_get_binding(argv[1]);
	if (reg_dev == NULL) {
		shell_error(sh, "regulator device %s not available", argv[1]);
		return -ENODEV;
	}

	ret = regulator_disable(reg_dev);
	if (ret < 0) {
		shell_error(sh, "failed  to disable regulator, error %d", ret);
		return ret;
	}
	shell_print(sh, "disabled regulator");
	return 0;
}

static int cmd_set_vol(const struct shell *sh, size_t argc, char **argv)
{
	int lvol, uvol, ret;
	const struct device *reg_dev;

	reg_dev = device_get_binding(argv[1]);
	if (reg_dev == NULL) {
		shell_error(sh, "regulator device %s not available", argv[1]);
		return -ENODEV;
	}

	lvol = strtol(argv[2], NULL, 10) * 1000;
	uvol = strtol(argv[3], NULL, 10) * 1000;
	shell_print(sh, "Setting range to %d-%d uV", lvol, uvol);
	ret = regulator_set_voltage(reg_dev, lvol, uvol);
	if (ret < 0) {
		shell_error(sh, "failed to set voltage, error %d", ret);
		return ret;
	}
	ret = regulator_get_voltage(reg_dev);
	if (ret < 0) {
		shell_error(sh, "failed to read voltage, error %d", ret);
		return ret;
	}
	shell_print(sh, "set voltage to %d uV", ret);
	return 0;
}

static int cmd_set_ilim(const struct shell *sh, size_t argc, char **argv)
{
	int lcur, ucur, ret;
	const struct device *reg_dev;

	reg_dev = device_get_binding(argv[1]);
	if (reg_dev == NULL) {
		shell_error(sh, "regulator device %s not available", argv[1]);
		return -ENODEV;
	}

	lcur = strtol(argv[2], NULL, 10) * 1000;
	ucur = strtol(argv[3], NULL, 10) * 1000;
	shell_print(sh, "Setting range to %d-%d uA", lcur, ucur);
	ret = regulator_set_current_limit(reg_dev, lcur, ucur);
	if (ret < 0) {
		shell_error(sh, "failed to set current, error %d", ret);
		return ret;
	}
	ret = regulator_get_current_limit(reg_dev);
	if (ret < 0) {
		shell_error(sh, "failed to read current, error %d", ret);
		return ret;
	}
	shell_print(sh, "set current limit to %d uA", ret);
	return 0;
}

static int cmd_set_mode(const struct shell *sh, size_t argc, char **argv)
{
	int mode, ret;
	const struct device *reg_dev;

	reg_dev = device_get_binding(argv[1]);
	if (reg_dev == NULL) {
		shell_error(sh, "regulator device %s not available", argv[1]);
		return -ENODEV;
	}

	mode = strtol(argv[2], NULL, 10);
	ret = regulator_set_mode(reg_dev, mode);
	if (ret < 0) {
		shell_error(sh, "failed to set mode, error %d", ret);
	}
	return ret;
}

static int cmd_set_mode_vol(const struct shell *sh, size_t argc, char **argv)
{
	int mode, lvol, uvol, ret;
	const struct device *reg_dev;

	reg_dev = device_get_binding(argv[1]);
	if (reg_dev == NULL) {
		shell_error(sh, "regulator device %s not available", argv[1]);
		return -ENODEV;
	}
	mode = strtol(argv[2], NULL, 10);
	lvol = strtol(argv[3], NULL, 10) * 1000;
	uvol = strtol(argv[4], NULL, 10) * 1000;
	ret = regulator_set_mode_voltage(reg_dev, mode, lvol, uvol);
	if (ret < 0) {
		shell_error(sh, "failed to set mode voltage, error %d", ret);
		return ret;
	}
	ret = regulator_get_mode_voltage(reg_dev, mode);
	if (ret < 0) {
		shell_error(sh, "failed to get mode voltage, error %d", ret);
		return ret;
	}
	shell_print(sh, "set voltage for mode %d to %d uV", mode, ret);
	return ret;
}

static int cmd_reg_mode_en(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *reg_dev;
	int ret, mode;

	reg_dev = device_get_binding(argv[1]);
	if (reg_dev == NULL) {
		shell_error(sh, "regulator device %s not available", argv[1]);
		return -ENODEV;
	}
	mode = strtol(argv[2], NULL, 10);

	ret = regulator_mode_enable(reg_dev, mode);
	if (ret < 0) {
		shell_error(sh, "failed to enable regulator for "
			"mode %d, error %d", mode, ret);
		return ret;
	}
	shell_print(sh, "enabled regulator for mode %d", mode);
	return 0;
}


static int cmd_reg_mode_dis(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *reg_dev;
	int ret, mode;

	reg_dev = device_get_binding(argv[1]);
	if (reg_dev == NULL) {
		shell_error(sh, "regulator device %s not available", argv[1]);
		return -ENODEV;
	}
	mode = strtol(argv[2], NULL, 10);

	ret = regulator_mode_disable(reg_dev, mode);
	if (ret < 0) {
		shell_error(sh, "failed to disable regulator for "
			"mode %d, error %d", mode, ret);
		return ret;
	}
	shell_print(sh, "disabled regulator for mode %d", mode);
	return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(regulator_set,
	SHELL_CMD_ARG(enable, NULL,
		"Enable regulator\n"
		"Usage: enable <device>", cmd_reg_en, 2, 0),
	SHELL_CMD_ARG(disable, NULL,
		"Disable regulator\n"
		"Usage: disable <device>", cmd_reg_dis, 2, 0),
	SHELL_CMD_ARG(set_vol, NULL, "Set voltage (in mV)\n"
		"Usage: set_vol <device> <low limit (uV)> <high limit (uV)>",
		cmd_set_vol, 4, 0),
	SHELL_CMD_ARG(set_current, NULL, "Set current limit( in mA)\n"
		"Usage: set_current <device> <low limit (uA)> <high limit (uA)>",
		cmd_set_ilim, 4, 0),
	SHELL_CMD_ARG(set_mode, NULL, "Set mode of regulator\n"
		"Usage: set_mode <device> <mode index>",
		cmd_set_mode, 3, 0),
	SHELL_CMD_ARG(set_mode_vol, NULL, "Set voltage for mode of regulator\n"
		"Usage: set_mode_vol <device> <mode index> "
		"<low limit (uV)> <high limit (uV)>",
		cmd_set_mode_vol, 5, 0),
	SHELL_CMD_ARG(mode_enable, NULL,
		"Enable regulator in given mode\n"
		"Usage: enable <device> <mode index>", cmd_reg_mode_en, 3, 0),
	SHELL_CMD_ARG(mode_disable, NULL,
		"Disable regulator in given mode\n"
		"Usage: disable <device> <mode index>", cmd_reg_mode_dis, 3, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(regulator, &regulator_set, "Regulator Management", NULL);
