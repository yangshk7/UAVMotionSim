"""Explicit native-platform GUI integration check; never collected by pytest."""

import argparse
import json
import sys
import time
import tkinter as tk
from pathlib import Path

from uavsim.config.loader import load_config
from uavsim.config.validator import Config

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "main" / "internal"))


def main():
    from control_panel import ControlPanel

    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", choices=("sitl", "px4"), required=True)
    parser.add_argument("--duration", type=float, default=20.48)
    parser.add_argument("--height", type=float, default=300)
    parser.add_argument("--passes", choices=(1, 2), type=int, default=1)
    parser.add_argument("--mode", choices=("simulate", "acceptance"), default="simulate")
    parser.add_argument("--navigation-source", choices=("shared_model", "flight_controller"), default="shared_model")
    args = parser.parse_args()
    root = tk.Tk()
    root.withdraw()
    app = ControlPanel(root)
    try:
        data = load_config(PROJECT / "main" / "config.yaml").model_dump()
        data["runtime"].update(backend=args.backend, external_navigation_source=args.navigation_source)
        data["simulation"].update(duration_mode="fixed", acquisition_duration_s=args.duration)
        data["trajectory"]["apc_height_m"] = args.height
        app.apply_config(Config.model_validate(data), PROJECT / "main" / "config.yaml")
        app.passes.set(str(args.passes))
        app.start(args.mode)
        assert app.process is not None
        print("JOB " + str(app.active_job), flush=True)
        deadline = time.monotonic()+1800
        while app.process is not None and time.monotonic() < deadline:
            root.update()
            time.sleep(.05)
        assert app.process is None, "External GUI timeout"
        expected = "partial" if args.mode == "acceptance" else "simulated"
        assert app.result and app.result["status"] == expected, app.log.get("1.0", "end")[-8000:]
        assert len(app.track_axes.lines) == 2
        assert len(app.pose_table.get_children()) == 5
        assert app.image is not None
        assert float(app.progress["value"]) == 100
        passes = 2 if args.mode == "acceptance" else args.passes
        assert len(app.pose_arrays) == passes
        output = Path(app.result["data_directory"])
        for i in range(1, passes+1):
            for stem in ("motion_truth", "motion_navigation", "flight_controller_navigation", "shared_navigation"):
                assert (output/f"{stem}_pass{i}.csv").exists()
        checks = {"result": app.result, "trajectory_lines": len(app.track_axes.lines),
                  "pose_rows": len(app.pose_table.get_children()), "loaded_passes": len(app.pose_arrays),
                  "progress": float(app.progress["value"]), "status": "passed"}
        (app.active_job/"gui_integration_check.json").write_text(
            json.dumps(checks, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(checks, ensure_ascii=False), flush=True)
    finally:
        if app.process is not None:
            app.stop()
        root.destroy()


if __name__ == "__main__":
    main()
