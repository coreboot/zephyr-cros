/*
 * Copyright (c) 2021 Tokita, Hiroshi <tokita.hiroshi@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file SoC configuration macros for the GigaDevice GD32VF103 processor
 */

#ifndef RISCV_GD32VF103_SOC_H_
#define RISCV_GD32VF103_SOC_H_

#include <soc_common.h>
#include <devicetree.h>

/* Timer configuration */
#define RISCV_MTIME_BASE      DT_REG_ADDR_BY_IDX(DT_NODELABEL(mtimer), 0)
#define RISCV_MTIMECMP_BASE   DT_REG_ADDR_BY_IDX(DT_NODELABEL(mtimer), 1)

#ifndef _ASMLANGUAGE
#include <toolchain.h>
#include <gd32vf103.h>
#endif  /* !_ASMLANGUAGE */

#endif  /* RISCV_GD32VF103_SOC_H */
