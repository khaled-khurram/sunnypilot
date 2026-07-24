#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="18.4"
fi

export STAGING_ROOT="/data/safe_staging"

# Tailscale persistence: /var is tmpfs on this device (confirmed 2026-07-24 --
# root's crontab is wiped every reboot), so cron can't be the boot hook. This
# script is the earliest persistent (/data-backed) thing sourced every boot.
[ -x /data/tailscale-state/start_tailscaled.sh ] && /data/tailscale-state/start_tailscaled.sh &
