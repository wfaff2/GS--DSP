#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ -f devel/setup.bash ]]; then
  # shellcheck disable=SC1091
  source devel/setup.bash
fi

exec python3 scripts/run_vw_three_controller_20seed.py \
  --jobs 16 \
  --obs 100 \
  --seed-start 1 \
  --seed-end 20 \
  --noise-seed 1 \
  --v-values 1.0,1.5,2.0,2.5,3.0 \
  --w-values 0.5,1.0,1.5,2.0 \
  --controllers inertial_frozen,noninertial_frozen,noninertial_stage \
  --active-uavs 1,2,3 \
  "$@"
