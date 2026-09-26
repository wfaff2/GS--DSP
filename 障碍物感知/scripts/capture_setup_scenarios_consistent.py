#!/usr/bin/env python3
from pathlib import Path
import os
import re
import signal
import subprocess
import time
from typing import Tuple

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent.parent
TEMPLATE_RVIZ = ROOT / "src" / "coni_mpc" / "rviz" / "world.rviz"
OUT_DIR = ROOT / "figures"

WINDOW_GEOMETRY_RE = re.compile(
    r'\s*(0x[0-9a-fA-F]+)\s.*?(\d+)x(\d+)\+(-?\d+)\+(-?\d+)\s+\+(-?\d+)\+(-?\d+)'
)

CAMERA = {
    "distance": 12.8,
    "focal_x": 0.0,
    "focal_y": 0.0,
    "focal_z": 0.4,
    "pitch": 1.34,
    "yaw": 4.22,
    "target_frame": "map",
}

SCENARIOS = [
    {
        "label": "a",
        "name": "circle",
        "png": OUT_DIR / "paper_setup_circle_consistent.png",
        "port": 11661,
        "wait_sec": 18.0,
        "launch_args": [
            "v:=0.5", "w:=0.5", "r:=1.0", "seed:=1", "kappa:=1.0",
            "obstacle_count:=0", "trajectory_laps:=20.0",
            "active_uavs_csv:=1,2,3",
            "exclude_uav0_from_simulation:=true",
            "exclude_uav0_from_all_metrics:=true",
            "car_trajectory_mode:=circle",
            "figure_eight_obstacles_enabled:=false",
            "safety_variant:=A0_no_cbf",
            "cbf_enabled:=false", "cbf_use_in_sim:=false",
            "ugv_rollout_mode:=stage", "use_rviz:=false",
        ],
    },
    {
        "label": "b",
        "name": "figure8",
        "png": OUT_DIR / "paper_setup_figure8_consistent.png",
        "port": 11662,
        "wait_sec": 48.0,
        "launch_args": [
            "v:=0.5", "w:=0.5", "r:=1.0", "seed:=1", "kappa:=1.0",
            "obstacle_count:=0", "trajectory_laps:=10.0",
            "active_uavs_csv:=1,2,3",
            "exclude_uav0_from_simulation:=true",
            "exclude_uav0_from_all_metrics:=true",
            "car_trajectory_mode:=figure_eight",
            "figure_eight_obstacles_enabled:=false",
            "safety_variant:=A0_no_cbf",
            "cbf_enabled:=false", "cbf_use_in_sim:=false",
            "ugv_rollout_mode:=stage", "use_rviz:=false",
        ],
    },
    {
        "label": "c",
        "name": "figure8_obs",
        "png": OUT_DIR / "paper_setup_figure8_obs_consistent.png",
        "port": 11663,
        "wait_sec": 48.0,
        "launch_args": [
            "v:=0.5", "w:=0.5", "r:=1.0", "seed:=1", "kappa:=1.0",
            "obstacle_count:=0", "trajectory_laps:=10.0",
            "active_uavs_csv:=1,2,3",
            "exclude_uav0_from_simulation:=true",
            "exclude_uav0_from_all_metrics:=true",
            "car_trajectory_mode:=figure_eight",
            "figure_eight_obstacles_enabled:=true",
            "safety_variant:=A2_soft_cbf",
            "cbf_enabled:=true", "cbf_use_in_sim:=true",
            "ugv_rollout_mode:=stage", "use_rviz:=false",
        ],
    },
]


def load_font(size: int):
    for path in [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
    ]:
        if Path(path).exists():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


def prepare_rviz_config(case_name: str) -> Path:
    text = TEMPLATE_RVIZ.read_text()
    replacements = {
        r"(\n      Distance: )[-0-9.eE]+": r"\g<1>{}".format(CAMERA["distance"]),
        r"(\n        X: )[-0-9.eE]+": r"\g<1>{}".format(CAMERA["focal_x"]),
        r"(\n        Y: )[-0-9.eE]+": r"\g<1>{}".format(CAMERA["focal_y"]),
        r"(\n        Z: )[-0-9.eE]+": r"\g<1>{}".format(CAMERA["focal_z"]),
        r"(\n      Pitch: )[-0-9.eE]+": r"\g<1>{}".format(CAMERA["pitch"]),
        r"(\n      Target Frame: ).*": r"\g<1>{}".format(CAMERA["target_frame"]),
        r"(\n      Yaw: )[-0-9.eE]+": r"\g<1>{}".format(CAMERA["yaw"]),
    }
    view_start = text.index("  Views:\n")
    geom_start = text.index("Window Geometry:\n")
    head = text[:view_start]
    view = text[view_start:geom_start]
    tail = text[geom_start:]
    for pattern, replacement in replacements.items():
        view = re.sub(pattern, replacement, view, count=1)
    rviz_path = Path("/tmp") / f"paper_setup_{case_name}_world.rviz"
    rviz_path.write_text(head + view + tail)
    return rviz_path


def kill_process_group(proc: subprocess.Popen) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        return
    try:
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass


def get_window_pid(window_id: str) -> int:
    try:
        out = subprocess.check_output(
            ["xprop", "-id", window_id, "_NET_WM_PID"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except subprocess.CalledProcessError:
        return -1
    if " = " not in out:
        return -1
    try:
        return int(out.split(" = ", 1)[1])
    except ValueError:
        return -1


def is_rviz_window(line: str) -> bool:
    return "RViz" in line and '"rviz"' in line


def find_rviz_window_id(pid: int, timeout_sec: float = 40.0) -> str:
    deadline = time.time() + timeout_sec
    best = None
    while time.time() < deadline:
        tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
        for line in tree.splitlines():
            if not is_rviz_window(line):
                continue
            match = WINDOW_GEOMETRY_RE.match(line)
            if not match:
                continue
            window_id = match.group(1)
            width = int(match.group(2))
            height = int(match.group(3))
            if get_window_pid(window_id) != pid:
                continue
            best = (window_id, width, height)
            if width >= 1200 and height >= 800:
                return window_id
        time.sleep(0.5)
    if best is not None:
        return best[0]
    raise RuntimeError(f"RViz window for pid {pid} not found")


def find_rviz_render_geometry(window_id: str) -> Tuple[int, int, int, int]:
    tree = subprocess.check_output(["xwininfo", "-id", window_id, "-tree"], text=True)
    candidates = []
    for line in tree.splitlines():
        match = WINDOW_GEOMETRY_RE.match(line)
        if not match:
            continue
        child_id = match.group(1)
        width = int(match.group(2))
        height = int(match.group(3))
        abs_x = int(match.group(6))
        abs_y = int(match.group(7))
        if child_id == window_id:
            continue
        if width < 700 or height < 600:
            continue
        candidates.append((width * height, abs_x, abs_y, width, height))
    if not candidates:
        raise RuntimeError("RViz render viewport not found")
    _, abs_x, abs_y, width, height = max(candidates)
    return abs_x, abs_y, width, height


def capture_window_region(abs_x: int, abs_y: int, width: int, height: int, out_path: Path) -> None:
    screenshot_path = Path("/tmp") / f"{out_path.stem}_screen.png"
    subprocess.check_call(["gnome-screenshot", "-f", str(screenshot_path)])
    image = Image.open(screenshot_path).convert("RGB")
    crop = image.crop((abs_x, abs_y, abs_x + width, abs_y + height))
    crop.save(out_path)


def run_case(case: dict) -> None:
    log_path = Path(f"/tmp/{case['name']}_setup_capture.log")
    rviz_config = prepare_rviz_config(case["name"])
    env = (
        "source /opt/ros/noetic/setup.bash && "
        "source devel/setup.bash && "
        "export DISABLE_ROS1_EOL_WARNINGS=1 && "
        f"export ROS_MASTER_URI=http://127.0.0.1:{case['port']} ROS_IP=127.0.0.1 ROS_HOSTNAME=127.0.0.1 && "
    )
    launch_cmd = (
        env
        + "roslaunch src/coni_mpc/launch/num_sim_non_one_point.launch "
        + " ".join(case["launch_args"])
    )
    rviz_cmd = env + f"exec rviz -d '{rviz_config}'"

    sim_proc = None
    rviz_proc = None
    with open(log_path, "w") as logf:
        sim_proc = subprocess.Popen(
            ["bash", "-lc", launch_cmd],
            cwd=str(ROOT),
            stdout=logf,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
    try:
        time.sleep(5.0)
        with open(log_path, "a") as logf:
            rviz_proc = subprocess.Popen(
                ["bash", "-lc", rviz_cmd],
                cwd=str(ROOT),
                stdout=logf,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
        window_id = find_rviz_window_id(rviz_proc.pid)
        time.sleep(case["wait_sec"])
        abs_x, abs_y, width, height = find_rviz_render_geometry(window_id)
        capture_window_region(abs_x, abs_y, width, height, case["png"])
        print(case["png"])
    finally:
        if rviz_proc is not None:
            kill_process_group(rviz_proc)
        kill_process_group(sim_proc)


def compose_final() -> None:
    panel_w = 540
    panel_h = 540
    panel_pad = 18
    outer_pad_x = 28
    outer_pad_y = 26
    gap = 24
    row_gap = 26
    label_margin = 14
    font = load_font(28)
    background = (255, 255, 255)
    border = (220, 220, 220)
    text = (0, 0, 0)
    canvas_w = outer_pad_x * 2 + 2 * panel_w + gap
    canvas_h = outer_pad_y * 2 + 2 * panel_h + row_gap
    canvas = Image.new("RGB", (canvas_w, canvas_h), background)
    draw = ImageDraw.Draw(canvas)

    positions = [
        ((canvas_w - panel_w) // 2, outer_pad_y),
        (outer_pad_x, outer_pad_y + panel_h + row_gap),
        (outer_pad_x + panel_w + gap, outer_pad_y + panel_h + row_gap),
    ]
    for case, (x, y) in zip(SCENARIOS, positions):
        image = Image.open(case["png"]).convert("RGB")
        image.thumbnail(
            (panel_w - 2 * panel_pad, panel_h - 2 * panel_pad),
            Image.LANCZOS if hasattr(Image, "LANCZOS") else Image.ANTIALIAS,
        )
        panel = Image.new("RGB", (panel_w, panel_h), background)
        px = (panel_w - image.width) // 2
        py = (panel_h - image.height) // 2
        panel.paste(image, (px, py))
        ImageDraw.Draw(panel).rectangle(
            [(0, 0), (panel_w - 1, panel_h - 1)], outline=border, width=1
        )
        canvas.paste(panel, (x, y))
        draw.text((x + label_margin, y + label_margin), f"({case['label']})", fill=text, font=font)

    png = OUT_DIR / "paper_fig1_scenarios.png"
    pdf = OUT_DIR / "paper_fig1_scenarios.pdf"
    canvas.save(png)
    canvas.save(pdf, "PDF", resolution=300.0)
    print(png)
    print(pdf)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for case in SCENARIOS:
        run_case(case)
    compose_final()


if __name__ == "__main__":
    main()
