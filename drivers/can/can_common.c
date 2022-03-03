/*
 * Copyright (c) 2019 Alexander Wachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/can.h>
#include <kernel.h>
#include <sys/util.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(can_common, CONFIG_CAN_LOG_LEVEL);

/* CAN sync segment is always one time quantum */
#define CAN_SYNC_SEG 1

static void can_msgq_put(struct zcan_frame *frame, void *arg)
{
	struct k_msgq *msgq = (struct k_msgq *)arg;
	int ret;

	__ASSERT_NO_MSG(msgq);

	ret = k_msgq_put(msgq, frame, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Msgq %p overflowed. Frame ID: 0x%x", arg, frame->id);
	}
}

int z_impl_can_add_rx_filter_msgq(const struct device *dev, struct k_msgq *msgq,
				  const struct zcan_filter *filter)
{
	const struct can_driver_api *api = dev->api;

	return api->add_rx_filter(dev, can_msgq_put, msgq, filter);
}

static int update_sampling_pnt(uint32_t ts, uint32_t sp, struct can_timing *res,
			       const struct can_timing *max,
			       const struct can_timing *min)
{
	uint16_t ts1_max = max->phase_seg1 + max->prop_seg;
	uint16_t ts1_min = min->phase_seg1 + min->prop_seg;
	uint32_t sp_calc;
	uint16_t ts1, ts2;

	ts2 = ts - (ts * sp) / 1000;
	ts2 = CLAMP(ts2, min->phase_seg2, max->phase_seg2);
	ts1 = ts - CAN_SYNC_SEG - ts2;

	if (ts1 > ts1_max) {
		ts1 = ts1_max;
		ts2 = ts - CAN_SYNC_SEG - ts1;
		if (ts2 > max->phase_seg2) {
			return -1;
		}
	} else if (ts1 < ts1_min) {
		ts1 = ts1_min;
		ts2 = ts - ts1;
		if (ts2 < min->phase_seg2) {
			return -1;
		}
	}

	res->prop_seg = CLAMP(ts1 / 2, min->prop_seg, max->prop_seg);
	res->phase_seg1 = ts1 - res->prop_seg;
	res->phase_seg2 = ts2;

	sp_calc = (CAN_SYNC_SEG + ts1) * 1000 / ts;

	return sp_calc > sp ? sp_calc - sp : sp - sp_calc;
}

/* Internal function to do the actual calculation */
static int can_calc_timing_int(uint32_t core_clock, struct can_timing *res,
			       const struct can_timing *min,
			       const struct can_timing *max,
			       uint32_t bitrate, uint16_t sp)
{
	uint32_t ts = max->prop_seg + max->phase_seg1 + max->phase_seg2 +
		   CAN_SYNC_SEG;
	uint16_t sp_err_min = UINT16_MAX;
	int sp_err;
	struct can_timing tmp_res;

	if (sp >= 1000 ||
	    (!IS_ENABLED(CONFIG_CAN_FD_MODE) && bitrate > 1000000) ||
	     (IS_ENABLED(CONFIG_CAN_FD_MODE) && bitrate > 8000000)) {
		return -EINVAL;
	}

	for (int prescaler = MAX(core_clock / (ts * bitrate), 1);
	     prescaler <= max->prescaler; ++prescaler) {
		if (core_clock % (prescaler * bitrate)) {
			/* No integer ts */
			continue;
		}

		ts = core_clock / (prescaler * bitrate);

		sp_err = update_sampling_pnt(ts, sp, &tmp_res,
					     max, min);
		if (sp_err < 0) {
			/* No prop_seg, seg1, seg2 combination possible */
			continue;
		}

		if (sp_err < sp_err_min) {
			sp_err_min = sp_err;
			res->prop_seg = tmp_res.prop_seg;
			res->phase_seg1 = tmp_res.phase_seg1;
			res->phase_seg2 = tmp_res.phase_seg2;
			res->prescaler = (uint16_t)prescaler;
			if (sp_err == 0) {
				/* No better result than a perfect match*/
				break;
			}
		}
	}

	if (sp_err_min) {
		LOG_DBG("SP error: %d 1/1000", sp_err_min);
	}

	return sp_err_min == UINT16_MAX ? -EINVAL : (int)sp_err_min;
}

int can_calc_timing(const struct device *dev, struct can_timing *res,
		    uint32_t bitrate, uint16_t sample_pnt)
{
	const struct can_driver_api *api = dev->api;
	uint32_t core_clock;
	int ret;

	ret = can_get_core_clock(dev, &core_clock);
	if (ret != 0) {
		return ret;
	}

	return can_calc_timing_int(core_clock, res, &api->timing_min,
				   &api->timing_max, bitrate, sample_pnt);
}

#ifdef CONFIG_CAN_FD_MODE
int can_calc_timing_data(const struct device *dev, struct can_timing *res,
			 uint32_t bitrate, uint16_t sample_pnt)
{
	const struct can_driver_api *api = dev->api;
	uint32_t core_clock;
	int ret;

	ret = can_get_core_clock(dev, &core_clock);
	if (ret != 0) {
		return ret;
	}

	return can_calc_timing_int(core_clock, res, &api->timing_min_data,
				   &api->timing_max_data, bitrate, sample_pnt);
}
#endif

int can_calc_prescaler(const struct device *dev, struct can_timing *timing,
		       uint32_t bitrate)
{
	uint32_t ts = timing->prop_seg + timing->phase_seg1 + timing->phase_seg2 +
		   CAN_SYNC_SEG;
	uint32_t core_clock;
	int ret;

	ret = can_get_core_clock(dev, &core_clock);
	if (ret != 0) {
		return ret;
	}

	timing->prescaler = core_clock / (bitrate * ts);

	return core_clock % (ts * timing->prescaler);
}
