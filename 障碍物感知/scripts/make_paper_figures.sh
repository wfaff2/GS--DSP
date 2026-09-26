#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SKILL_DIR="/home/jjm/.codex/skills/ieee-robotics-figure-style"
SRC_DIR="${ROOT_DIR}/figures_src"
FIG_DIR="${ROOT_DIR}/figures"

mkdir -p "${SRC_DIR}/shared" "${FIG_DIR}"
cp "${SKILL_DIR}/shared/robotics_figure_colors.tex" "${SRC_DIR}/shared/"
cp "${SKILL_DIR}/shared/robotics_figure_styles.tex" "${SRC_DIR}/shared/"
cp "${SKILL_DIR}/shared/robotics_figure_macros.tex" "${SRC_DIR}/shared/"

"${SKILL_DIR}/scripts/compile_tikz_figure.sh" \
  "${SRC_DIR}/paper_fig1_scenario_overview_standalone.tex" \
  "${FIG_DIR}"

"${SKILL_DIR}/scripts/export_tikz_png.sh" \
  "${FIG_DIR}/paper_fig1_scenario_overview_standalone.pdf" \
  "${FIG_DIR}/paper_fig1_scenario_overview"

if [ -f "${FIG_DIR}/paper_fig1_scenario_overview-1.png" ]; then
  mv "${FIG_DIR}/paper_fig1_scenario_overview-1.png" "${FIG_DIR}/paper_fig1_scenario_overview.png"
fi

python3 "${ROOT_DIR}/scripts/plot_paper_fig2_pure_tracking.py"
python3 "${ROOT_DIR}/scripts/plot_paper_fig3_cbf_main.py"
python3 "${ROOT_DIR}/scripts/plot_paper_fig4_kp4_seed046_case.py"
