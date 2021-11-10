/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <pm/device.h>
#include <pm/device_runtime.h>
#include <ztest.h>
#include <ksched.h>
#include <kernel.h>
#include <pm/pm.h>
#include "dummy_driver.h"

#define SLEEP_MSEC 100
#define SLEEP_TIMEOUT K_MSEC(SLEEP_MSEC)

/* for checking power suspend and resume order between system and devices */
static bool enter_low_power;
static bool notify_app_entry;
static bool notify_app_exit;
static bool set_pm;
static bool leave_idle;
static bool idle_entered;
static bool testing_device_runtime;

static const struct device *dev;
static struct dummy_driver_api *api;

void pm_power_state_set(struct pm_state_info info)
{
	/* at this point, notify_pm_state_entry() implemented in
	 * this file has been called and set_pm should have been set
	 */
	zassert_true(set_pm == true,
		     "Notification to enter suspend was not sent to the App");

	/* this function is called after devices enter low power state */
	enum pm_device_state device_power_state;
	pm_device_state_get(dev, &device_power_state);

	if (testing_device_runtime) {
		/* If device runtime is enable, the device should still be
		 * active
		 */
		zassert_true(device_power_state == PM_DEVICE_STATE_ACTIVE, NULL);
	} else {
		/* at this point, devices have been deactivated */
		zassert_false(device_power_state == PM_DEVICE_STATE_ACTIVE, NULL);
	}

	/* this function is called when system entering low power state, so
	 * parameter state should not be PM_STATE_ACTIVE
	 */
	zassert_false(info.state == PM_STATE_ACTIVE,
		      "Entering low power state with a wrong parameter");
}

void pm_power_state_exit_post_ops(struct pm_state_info info)
{
	/* pm_system_suspend is entered with irq locked
	 * unlock irq before leave pm_system_suspend
	 */
	irq_unlock(0);
}

/* Our PM policy handler */
struct pm_state_info pm_policy_next_state(uint8_t cpu, int ticks)
{
	struct pm_state_info info = {};

	ARG_UNUSED(cpu);

	/* make sure this is idle thread */
	zassert_true(z_is_idle_thread_object(_current), NULL);
	zassert_true(ticks == _kernel.idle, NULL);
	idle_entered = true;

	if (enter_low_power) {
		enter_low_power = false;
		notify_app_entry = true;
		info.state = PM_STATE_SUSPEND_TO_IDLE;
	} else {
		/* only test pm_policy_next_state()
		 * no PM operation done
		 */
		info.state = PM_STATE_ACTIVE;
	}
	return info;
}

/* implement in application, called by idle thread */
static void notify_pm_state_entry(enum pm_state state)
{
	enum pm_device_state device_power_state;

	/* enter suspend */
	zassert_true(notify_app_entry == true,
		     "Notification to enter suspend was not sent to the App");
	zassert_true(z_is_idle_thread_object(_current), NULL);
	zassert_equal(state, PM_STATE_SUSPEND_TO_IDLE, NULL);

	pm_device_state_get(dev, &device_power_state);
	if (testing_device_runtime) {
		/* If device runtime is enable, the device should still be
		 * active
		 */
		zassert_true(device_power_state == PM_DEVICE_STATE_ACTIVE, NULL);
	} else {
		/* at this point, devices should not be active */
		zassert_false(device_power_state == PM_DEVICE_STATE_ACTIVE, NULL);
	}
	set_pm = true;
	notify_app_exit = true;
}

/* implement in application, called by idle thread */
static void notify_pm_state_exit(enum pm_state state)
{
	enum pm_device_state device_power_state;

	/* leave suspend */
	zassert_true(notify_app_exit == true,
		     "Notification to leave suspend was not sent to the App");
	zassert_true(z_is_idle_thread_object(_current), NULL);
	zassert_equal(state, PM_STATE_SUSPEND_TO_IDLE, NULL);

	/* at this point, devices are active again*/
	pm_device_state_get(dev, &device_power_state);
	zassert_equal(device_power_state, PM_DEVICE_STATE_ACTIVE, NULL);
	leave_idle = true;

}

/*
 * @brief test power idle
 *
 * @details
 *  - The global idle routine executes when no other work is available
 *  - The idle routine provide a timeout parameter to the suspend routine
 *    indicating the amount of time guaranteed to expire before the next
 *    timeout, pm_policy_next_state() handle this parameter.
 *  - In this case, pm_policy_next_sate() return PM_STATE_ACTIVE,
 *    so there is no low power operation happen.
 *
 * @see pm_policy_next_state()
 *
 * @ingroup power_tests
 */
void test_power_idle(void)
{
	TC_PRINT("give way to idle thread\n");
	k_sleep(SLEEP_TIMEOUT);
	zassert_true(idle_entered, "Never entered idle thread");
}

/*
 * @brief test power state transition
 *
 * @details
 *  - The system support control of power state ordering between
 *    subsystems and devices
 *  - The application can control system power state transitions in idle thread
 *    through pm_notify_pm_state_entry and pm_notify_pm_state_exit
 *
 * @see pm_notify_pm_state_entry(), pm_notify_pm_state_exit()
 *
 * @ingroup power_tests
 */
void test_power_state_trans(void)
{
	int ret;

	enter_low_power = true;

	ret = pm_device_runtime_disable(dev);
	zassert_true(ret == 0, "Failed to disable device runtime PM");

	/* give way to idle thread */
	k_sleep(SLEEP_TIMEOUT);
	zassert_true(leave_idle, NULL);

	pm_device_runtime_enable(dev);
}

/*
 * @brief notification between system and device
 *
 * @details
 *  - device driver notify its power state change by pm_device_runtime_get and
 *    pm_device_runtime_put_async
 *  - system inform device system power state change through device interface
 *    pm_action_cb
 *
 * @see pm_device_runtime_get(), pm_device_runtime_put_async(),
 *      pm_device_state_set(), pm_device_state_get()
 *
 * @ingroup power_tests
 */
void test_power_state_notification(void)
{
	int ret;
	enum pm_device_state device_power_state;

	enter_low_power = true;

	ret = api->open(dev);
	zassert_true(ret == 0, "Fail to open device");

	pm_device_state_get(dev, &device_power_state);
	zassert_equal(device_power_state, PM_DEVICE_STATE_ACTIVE, NULL);


	/* The device should be kept active even when the system goes idle */
	testing_device_runtime = true;

	k_sleep(SLEEP_TIMEOUT);
	zassert_true(leave_idle, NULL);

	api->close(dev);
	pm_device_state_get(dev, &device_power_state);
	zassert_equal(device_power_state, PM_DEVICE_STATE_SUSPENDED, NULL);
}

void test_main(void)
{
	struct pm_notifier notifier = {
		.state_entry = notify_pm_state_entry,
		.state_exit = notify_pm_state_exit,
	};

	pm_notifier_register(&notifier);
	dev = device_get_binding(DUMMY_DRIVER_NAME);
	api = (struct dummy_driver_api *)dev->api;

	ztest_test_suite(power_management_test,
			 ztest_1cpu_unit_test(test_power_idle),
			 ztest_1cpu_unit_test(test_power_state_trans),
			 ztest_1cpu_unit_test(test_power_state_notification));
	ztest_run_test_suite(power_management_test);
	pm_notifier_unregister(&notifier);
}
