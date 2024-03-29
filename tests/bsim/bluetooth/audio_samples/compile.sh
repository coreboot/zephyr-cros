#!/usr/bin/env bash
# Copyright 2023 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Compile all the applications needed by the bsim tests in these subfolders

#set -x #uncomment this line for debugging
set -ue

: "${ZEPHYR_BASE:?ZEPHYR_BASE must be set to point to the zephyr root directory}"

source ${ZEPHYR_BASE}/tests/bsim/compile.source

if [ "${BOARD_TS}" == "nrf5340bsim_nrf5340_cpuapp" ]; then
  app=samples/bluetooth/unicast_audio_server sysbuild=1 compile
  app=samples/bluetooth/broadcast_audio_source sysbuild=1 compile
  app=tests/bsim/bluetooth/audio_samples/unicast_audio_client sysbuild=1 compile
  app=tests/bsim/bluetooth/audio_samples/broadcast_audio_sink sysbuild=1 \
    conf_file=${ZEPHYR_BASE}/samples/bluetooth/broadcast_audio_sink/prj.conf \
    exe_name=bs_${BOARD_TS}_${app}_prj_conf sysbuild=1 compile
else
  app=samples/bluetooth/unicast_audio_server conf_overlay=overlay-bt_ll_sw_split.conf \
    exe_name=bs_${BOARD_TS}_${app}_prj_conf sysbuild=1 compile
  app=samples/bluetooth/broadcast_audio_source conf_overlay=overlay-bt_ll_sw_split.conf \
    exe_name=bs_${BOARD_TS}_${app}_prj_conf sysbuild=1 compile
  app=tests/bsim/bluetooth/audio_samples/unicast_audio_client \
    conf_overlay=${ZEPHYR_BASE}/samples/bluetooth/unicast_audio_client/overlay-bt_ll_sw_split.conf \
    exe_name=bs_${BOARD_TS}_${app}_prj_conf sysbuild=1 compile
  app=tests/bsim/bluetooth/audio_samples/broadcast_audio_sink \
    conf_file=${ZEPHYR_BASE}/samples/bluetooth/broadcast_audio_sink/prj.conf \
    conf_overlay=${ZEPHYR_BASE}/samples/bluetooth/broadcast_audio_sink/overlay-bt_ll_sw_split.conf \
    exe_name=bs_${BOARD_TS}_${app}_prj_conf sysbuild=1 compile
fi

wait_for_background_jobs
