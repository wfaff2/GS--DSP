#!/usr/bin/env python3
from pathlib import Path
import os
import re
import signal
import struct
import subprocess
import time

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent.parent
OUT_FULL = ROOT / "figures" / "rviz_kp4_clean_figure8_stage_seed68_world_full.png"
OUT_CROP = ROOT / "figures" / "rviz_kp4_clean_figure8_stage_seed68_world.png"
XWD_TMP = Path("/tmp/rviz_kp4_clean_figure8_stage_seed68_world.xwd")
LOG = Path("/tmp/rviz_kp4_clean_figure8_stage_seed68_world.log")


def find_rviz_window_id() -> str:
    tree = subprocess.check_output(["bash", "-lc", "xwininfo -root -tree"], text=True)
    for line in tree.splitlines():
        if "world.rviz - RViz" in line or '"RViz"' in line:
            m = re.match(r"\\s*(0x[0-9a-fA-F]+)\\s", line)
            if m:
                return m.group(1)
    raise RuntimeError("RViz window not found")


def xwd_to_bgr(path: Path) -> np.ndarray:
    with open(path, "rb") as f:
        header = struct.unpack(">25I", f.read(100))
        header_size = header[0]
        width = header[4]
        height = header[5]
        bytes_per_line = header[12]
        ncolors = header[19]
        f.seek(header_size + ncolors * 12)
        raw = f.read(bytes_per_line * height)
    arr = np.frombuffer(raw, dtype=np.uint8).reshape(height, bytes_per_line)
    arr = arr[:, : width * 4].reshape(height, width, 4)
    return arr[:, :, :3].copy()


def main() -> None:
    cmd = (
        "source /opt/ros/noetic/setup.bash && "
        "source devel/setup.bash && "
        "roslaunch src/coni_mpc/launch/num_sim_non_one_point.launch "
        "v:=2.0 w:=2.0 r:=1.0 seed:=68 kappa:=1.0 obstacle_count:=0 "
        "active_uavs_csv:='1,2,3' exclude_uav0_from_simulation:=true "
        "exclude_uav0_from_all_metrics:=true car_trajectory_mode:=figure_eight "
        "figure_eight_obstacles_enabled:=false safety_variant:=A0_no_cbf "
        "cbf_enabled:=false cbf_use_in_sim:=false ugv_rollout_mode:=stage "
        "use_rviz:=true rviz_view:=world"
    )
    with open(LOG, "w") as logf:
        proc = subprocess.Popen(
            ["bash", "-lc", cmd],
            cwd=str(ROOT),
            stdout=logf,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
    try:
        time.sleep(20.0)
        wid = find_rviz_window_id()
        subprocess.check_call(["xwd", "-silent", "-id", wid, "-out", str(XWD_TMP)])
        img = xwd_to_bgr(XWD_TMP)
        OUT_FULL.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(OUT_FULL), img)
        # Crop the RViz render viewport and trim most UI chrome.
        crop = img[58:760, 422:1345]
        cv2.imwrite(str(OUT_CROP), crop)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            pass
        try:
            proc.wait(timeout=5)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass


if __name__ == "__main__":
    main()
