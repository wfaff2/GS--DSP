#!/usr/bin/env python3
"""Aggregate M2.5 frame logs into six tables, nine figures, and a verdict."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def p95(series):
    values = pd.to_numeric(series, errors="coerce").dropna()
    return np.nan if values.empty else float(np.percentile(values, 95))


def stats(series):
    values = pd.to_numeric(series, errors="coerce").dropna()
    if values.empty:
        return (np.nan,) * 4
    return values.mean(), np.sqrt(np.mean(values**2)), p95(values), values.max()


def save_table(table, root, name):
    table.to_csv(root / f"table_{name}.csv", index=False)
    return table


def plot_lines(groups, x, ys, labels, path, ylabel):
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for name, group in groups:
        order = group.sort_values(x)
        for y, label in zip(ys, labels):
            ax.plot(order[x], order[y], label=f"{name}: {label}")
    ax.set_xlabel(x)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("result_root", type=Path)
    args = parser.parse_args()
    root = args.result_root.resolve()
    plots = root / "plots"
    plots.mkdir(parents=True, exist_ok=True)
    files = sorted((root / "frames").glob("*.csv"))
    if not files:
        raise SystemExit("No frame CSV files found")
    data = pd.concat([pd.read_csv(path) for path in files], ignore_index=True)
    data = data[~data.run_id.str.startswith("smoke_")].copy()
    data["depth_cbf_ms"] = data.T_depth_cbf_total_ms
    data["relative_fov_angle_deg"] = np.abs(data.expected_obstacle_bearing_deg)
    data["raw_gradient_heading_deg"] = np.degrees(np.arctan2(data.nraw_y, data.nraw_x))
    data["fit_gradient_heading_deg"] = np.degrees(np.arctan2(data.nfit_y, data.nfit_x))
    nominal = data[~data.motion.isin(["blind", "fov"])]

    rows = []
    for scene, group in data.groupby("scene"):
        errors = group.distance_signed_error.dropna()
        mae, rmse, q95, maximum = stats(errors.abs())
        rows.append(dict(scene=scene, mean_signed_error=errors.mean(), MAE=mae,
                         RMSE=rmse, median_abs_error=errors.abs().median(),
                         P95_abs_error=q95, P99_abs_error=p95(errors.abs()) if len(errors) < 2 else np.percentile(errors.abs(), 99),
                         max_abs_error=maximum, sample_count=len(errors), frames=len(group)))
    table1 = save_table(pd.DataFrame(rows), root, "1_lidar_surface_distance")

    rows = []
    for scene, group in data.groupby("scene"):
        valid = group[group.valid == 1]
        center = valid.h_center_error.dropna()
        rows.append(dict(scene=scene,
            fit_RMSE_mean=valid.rmse.mean(), fit_RMSE_P95=p95(valid.rmse),
            fit_RMSE_max=valid.rmse.max(), fit_max_error_max=valid.max_abs_error.max(),
            center_mean_signed=center.mean(), center_MAE=center.abs().mean(),
            center_RMSE=np.sqrt(np.mean(center**2)) if len(center) else np.nan,
            center_P95_abs=p95(center.abs()), center_max_abs=center.abs().max(),
            center_sample_count=len(center)))
    table2 = save_table(pd.DataFrame(rows), root, "2_regression_accuracy")

    rows = []
    for scene, group in data.groupby("scene"):
        values = group.signed_boundary_shift.dropna()
        tolerance = group.non_conservative_tolerance.dropna().iloc[0] if len(group.non_conservative_tolerance.dropna()) else .001
        rows.append(dict(scene=scene, mean_signed_shift=values.mean(),
            MAE=values.abs().mean(), RMSE=np.sqrt(np.mean(values**2)) if len(values) else np.nan,
            P95_abs_error=p95(values.abs()),
            Max=values.abs().max(),
            tolerance=tolerance,
            non_conservative_ratio=(values > tolerance).mean() if len(values) else np.nan,
            boundary_frames=len(values)))
    table3 = save_table(pd.DataFrame(rows), root, "3_safety_boundary")

    rows = []
    for scene, group in data.groupby("scene"):
        angle = group.gradient_angle_deg.dropna()
        raw_jump = group.raw_gradient_jump_deg.dropna()
        fit_jump = group.fit_gradient_jump_deg.dropna()
        rows.append(dict(scene=scene, mean_gradient_angle=angle.mean(),
            median_gradient_angle=angle.median(), P95_angle=p95(angle),
            max_angle=angle.max(), gradient_samples=len(angle),
            raw_jump_mean=raw_jump.mean(), raw_jump_P95=p95(raw_jump),
            raw_jump_max=raw_jump.max(), fit_jump_mean=fit_jump.mean(),
            fit_jump_P95=p95(fit_jump), fit_jump_max=fit_jump.max()))
    table4 = save_table(pd.DataFrame(rows), root, "4_gradient_smoothness")

    rows = []
    for scene, group in data.groupby("scene"):
        row = {"scene": scene}
        for prefix, column in [("preprocess", "T_preprocess_ms"),
                               ("depth_cbf", "depth_cbf_ms"),
                               ("pipeline", "pipeline_ms")]:
            row[f"{prefix}_mean"] = group[column].mean()
            row[f"{prefix}_median"] = group[column].median()
            row[f"{prefix}_P95"] = p95(group[column])
            row[f"{prefix}_P99"] = np.percentile(group[column].dropna(), 99)
            row[f"{prefix}_max"] = group[column].max()
        row["sample_count"] = group.pipeline_ms.notna().sum()
        row["gt10ms_count"] = (group.pipeline_ms > 10).sum()
        row["gt10ms_rate"] = (group.pipeline_ms > 10).mean()
        row["gt20ms_count"] = (group.pipeline_ms > 20).sum()
        row["gt20ms_rate"] = (group.pipeline_ms > 20).mean()
        rows.append(row)
    table5 = save_table(pd.DataFrame(rows), root, "5_runtime")

    rows = []
    for scene, group in data.groupby("scene"):
        counts = group.failure_reason.value_counts(dropna=False)
        base = dict(scene=scene, frames=len(group), valid_rate=group.valid.mean())
        for reason in ["VALID", "EMPTY_CLOUD", "INSUFFICIENT_SUPPORT",
                       "RANK_DEFICIENT", "CONDITION_NUMBER_FAIL", "RMSE_FAIL",
                       "MAX_ERROR_FAIL", "SYNC_REJECT"]:
            base[f"{reason}_count"] = int(counts.get(reason, 0))
            base[f"{reason}_rate"] = counts.get(reason, 0) / len(group)
        rows.append(base)
    table6 = save_table(pd.DataFrame(rows), root, "6_barrier_availability")

    sync_rows = []
    for scene, group in data.groupby("scene"):
        for actor, column in [("quad", "quad_cloud_offset_ms"),
                              ("car", "car_cloud_offset_ms")]:
            values = group[column].abs().dropna()
            sync_rows.append(dict(scene=scene, actor=actor,
                mean_abs_offset_ms=values.mean(), P95_abs_offset_ms=p95(values),
                max_abs_offset_ms=values.max(), sample_count=len(values)))
    sync_table = pd.DataFrame(sync_rows)
    sync_table.to_csv(root / "sync_summary.csv", index=False)

    switch_events = data[(data.scene == "A4_switch") &
                         (data.nearest_changed == 1)][
        ["run_id", "cloud_stamp", "nearest_obstacle_id", "uav_x", "uav_y"]]
    switch_events.to_csv(root / "switch_events.csv", index=False)

    blind = data[data.motion == "blind"].copy()
    blind["distance_bin"] = pd.cut(blind.d_GT,
        [-np.inf, .5, 1.0, 1.5, 2.0, np.inf], right=False)
    blind_table = blind.groupby(["min_raylength", "distance_bin"],
        observed=True).agg(frames=("frame_index", "size"),
        lidar_return_rate=("d_lidar", lambda x: x.notna().mean()),
        valid_rate=("valid", "mean")).reset_index()
    blind_table.to_csv(root / "blind_zone_summary.csv", index=False)
    fov = data[data.motion == "fov"].copy()
    fov["angle_bin"] = pd.cut(fov.relative_fov_angle_deg,
        [0, 45, 90, 135, 180], include_lowest=True)
    fov_table = fov.groupby("angle_bin", observed=True).agg(
        frames=("frame_index", "size"),
        lidar_return_rate=("d_lidar", lambda x: x.notna().mean()),
        valid_rate=("valid", "mean")).reset_index()
    fov_table.to_csv(root / "fov_summary.csv", index=False)

    examples = [(name, group) for name, group in data.groupby("run_id")
                if name in {"A1_plane_front", "A2_cylinder_front"}]
    plot_lines(examples, "frame_index", ["d_lidar", "d_GT"],
               ["LiDAR", "GT"], plots / "A_distance_lidar_vs_gt.png", "distance (m)")
    plot_lines(examples, "frame_index", ["abs_d_lidar_d_GT"], ["absolute error"],
               plots / "B_distance_error.png", "absolute error (m)")
    plot_lines(examples, "frame_index", ["h_raw", "h_fit"], ["raw", "fit"],
               plots / "C_h_raw_vs_fit.png", "barrier value (m^2)")
    plot_lines(list(data.groupby("scene")), "frame_index", ["signed_boundary_shift"],
               ["fit - raw"], plots / "D_boundary_shift.png", "signed shift (m)")
    plot_lines(list(data.groupby("scene")), "frame_index", ["gradient_angle_deg"],
               ["angle"], plots / "E_gradient_angle.png", "angle (deg)")
    switch = data[data.scene.isin(["A3_corner", "A4_switch"])]
    switch = switch[switch.motion == "switching"]
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    for name, group in switch.groupby("run_id"):
        order = group.sort_values("frame_index")
        axes[0].plot(order.frame_index, order.raw_gradient_jump_deg,
                     label=f"{name}: raw")
        axes[0].plot(order.frame_index, order.fit_gradient_jump_deg,
                     label=f"{name}: fit")
        axes[1].plot(order.frame_index, order.raw_gradient_heading_deg,
                     label=f"{name}: raw")
        axes[1].plot(order.frame_index, order.fit_gradient_heading_deg,
                     label=f"{name}: fit")
    axes[0].set_ylabel("angular jump (deg)")
    axes[1].set_ylabel("gradient heading (deg)")
    axes[1].set_xlabel("frame_index")
    for axis in axes:
        axis.grid(True, alpha=.3)
        axis.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(plots / "F_switching_gradient_jump.png", dpi=160)
    plt.close(fig)
    plot_lines(list(data.groupby("scene")), "frame_index",
               ["T_preprocess_ms", "depth_cbf_ms", "pipeline_ms"],
               ["preprocess", "fit", "pipeline"], plots / "G_runtime.png", "time (ms)")
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for name, group in blind.groupby("run_id"):
        order = group.sort_values("frame_index")
        axes[0].plot(order.frame_index, order.d_GT, label=f"{name}: GT")
        axes[0].plot(order.frame_index, order.d_lidar, label=f"{name}: LiDAR")
        axes[1].step(order.frame_index, order.valid, where="post", label=name)
    blind_min_ray = blind.min_raylength.dropna()
    if len(blind_min_ray):
        min_ray = float(blind_min_ray.iloc[0])
        axes[0].axhline(min_ray, linestyle="--",
                       label=f"min_raylength={min_ray:g} m")
    axes[0].axhline(0.15, linestyle=":", label="d_safe=0.15 m")
    axes[0].set_ylabel("distance (m)")
    axes[1].set_ylabel("barrier valid")
    axes[1].set_xlabel("frame_index")
    axes[1].set_ylim(-.05, 1.05)
    for axis in axes:
        axis.grid(True, alpha=.3)
        axis.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(plots / "H_blind_zone.png", dpi=160)
    plt.close(fig)
    plot_lines(list(data.groupby("run_id")), "frame_index", ["valid"], ["valid"],
               plots / "I_valid_timeline.png", "valid")

    criterion_rows = [
        ("all_runs_recorded", len(data.run_id.unique()), 19, ">="),
        ("nominal_valid_rate", nominal.valid.mean(), .95, ">="),
        ("nominal_lidar_P95_m", p95(nominal.abs_d_lidar_d_GT), .15, "<="),
        ("nominal_center_fit_P95_m2", p95(nominal.abs_h_fit_h_raw), .05, "<="),
        ("nominal_boundary_P95_m", p95(nominal.signed_boundary_shift.abs()), .10, "<="),
        ("nominal_gradient_P95_deg", p95(nominal.gradient_angle_deg), 15.0, "<="),
        ("nominal_pipeline_P95_ms", p95(nominal.pipeline_ms), 20.0, "<="),
        ("sync_reject_count", (data.failure_reason == "SYNC_REJECT").sum(), 0, "=="),
    ]
    criteria = {name: (actual >= limit if op == ">=" else
                       actual <= limit if op == "<=" else actual == limit)
                for name, actual, limit, op in criterion_rows}
    pd.DataFrame([{"criterion": name, "actual": actual, "limit": limit,
                   "operator": op, "pass": criteria[name]}
                  for name, actual, limit, op in criterion_rows]).to_csv(
                      root / "acceptance_criteria.csv", index=False)
    verdict = "M2.5 PASS" if all(criteria.values()) else "M2.5 NOT READY"
    summary = ["# Depth-CBF M2.5 quantitative validation", "", verdict, "",
               "## Acceptance criteria", ""]
    summary += [f"- [{'x' if criteria[name] else ' '}] {name}: "
                f"actual={actual:.6g}, required {op} {limit:.6g}"
                for name, actual, limit, op in criterion_rows]
    for title, table in [("Table 1 - LiDAR surface-distance accuracy", table1),
                         ("Table 2 - Depth-CBF regression accuracy", table2),
                         ("Table 3 - Safety-boundary accuracy", table3),
                         ("Table 4 - Gradient accuracy and smoothness", table4),
                         ("Table 5 - Runtime", table5),
                         ("Table 6 - Barrier availability", table6)]:
        summary += ["", f"## {title}", "", table.to_markdown(index=False)]
    summary += ["", "## Blind-zone characterization", "",
                blind_table.to_markdown(index=False), "",
                "## 360-degree azimuth-coverage characterization", "",
                fov_table.to_markdown(index=False)]
    summary += ["", "## Synchronization offsets", "",
                sync_table.to_markdown(index=False), "",
                "## A4 GT obstacle switch events", "",
                switch_events.to_markdown(index=False)]
    (root / "summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
    pd.concat([table.assign(table=i + 1) for i, table in enumerate(
        [table1, table2, table3, table4, table5, table6])],
        ignore_index=True, sort=False).to_csv(root / "summary.csv", index=False)
    print(verdict)
    print(root / "summary.md")


if __name__ == "__main__":
    main()
