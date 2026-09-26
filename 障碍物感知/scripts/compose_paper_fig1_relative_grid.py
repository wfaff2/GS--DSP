#!/usr/bin/env python3
from pathlib import Path

import matplotlib.image as mpimg
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parent.parent
FIG_ROOT = ROOT / "figures"
OUT_PREFIX = FIG_ROOT / "paper_fig1_relative_grid"

PANELS = [
    ("a", FIG_ROOT / "circle_kp1_min_disadv.png"),
    ("b", FIG_ROOT / "circle_kp2_min_disadv.png"),
    ("c", FIG_ROOT / "circle_kp3_min_disadv.png"),
    ("d", FIG_ROOT / "circle_kp4_min_disadv.png"),
    ("e", FIG_ROOT / "circle_kp6_min_disadv.png"),
    ("f", FIG_ROOT / "figure8_clean_kp1_max_adv.png"),
    ("g", FIG_ROOT / "figure8_clean_kp2_max_adv.png"),
    ("h", FIG_ROOT / "figure8_clean_kp3_max_adv.png"),
    ("i", FIG_ROOT / "figure8_clean_kp4_max_adv.png"),
    ("j", FIG_ROOT / "figure8_clean_kp6_max_adv.png"),
    ("k", FIG_ROOT / "figure8_obs_kp1_max_adv.png"),
    ("l", FIG_ROOT / "figure8_obs_kp2_max_adv.png"),
    ("m", FIG_ROOT / "figure8_obs_kp3_max_adv.png"),
    ("n", FIG_ROOT / "figure8_obs_kp4_max_adv.png"),
    ("o", FIG_ROOT / "figure8_obs_kp6_max_adv.png"),
]


def main() -> None:
    fig, axes = plt.subplots(3, 5, figsize=(21.0, 12.2), constrained_layout=True)
    for ax, (label, path) in zip(axes.flat, PANELS):
        img = mpimg.imread(path)
        ax.imshow(img)
        ax.axis("off")
        ax.text(
            0.02,
            0.98,
            f"({label})",
            transform=ax.transAxes,
            ha="left",
            va="top",
            fontsize=14,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none", alpha=0.88),
        )

    fig.savefig(OUT_PREFIX.with_suffix(".png"), dpi=300, bbox_inches="tight")
    fig.savefig(OUT_PREFIX.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(fig)
    print(f"output_png={OUT_PREFIX.with_suffix('.png')}")
    print(f"output_pdf={OUT_PREFIX.with_suffix('.pdf')}")


if __name__ == "__main__":
    main()
