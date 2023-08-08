#!/usr/bin/env bash
set -eu

shdir="$(realpath "$(dirname "${BASH_SOURCE[0]}")")"
cd "${shdir}"

args_all=(-s=ccc_value -D=2)
args_dev=(-v=2 -RealEncryption=1 -testid=the_test)
sim_seconds=60

bin_dir="${BSIM_OUT_PATH}/bin"
BOARD="${BOARD:-nrf52_bsim}"

compile_path="${bin_dir}/bs_${BOARD}_"
compile_path+="$(realpath --relative-to "$(west topdir)"/zephyr prj.conf | tr /. _)"

echo "Simulation time: $sim_seconds seconds"

# bs_2G4_phy_v1 requires pwd to at its location
cd "${BSIM_OUT_PATH}/bin"

("${compile_path}" "${args_all[@]}" "${args_dev[@]}" -d=0 || echo d0 $?) &
("${compile_path}" "${args_all[@]}" "${args_dev[@]}" -d=1 || echo d1 $?) &
(./bs_2G4_phy_v1 "${args_all[@]}" -v=6 -sim_length=$((sim_seconds * 10 ** 6)) || echo phy $?) &

wait
