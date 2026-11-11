"""Explicit real-platform all-chain GUI check; fixed short baseline, no checks skipped."""

import argparse
import json
import shutil
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
    parser.add_argument("--review", type=Path, help="Replay completion of an existing job without rerunning flights")
    args = parser.parse_args()
    root = tk.Tk()
    root.withdraw()
    app = ControlPanel(root)
    try:
        data = load_config(PROJECT / "main" / "config.yaml").model_dump()
        data["simulation"].update(duration_mode="fixed", acquisition_duration_s=2, pre_stabilization_s=1)
        data["trajectory"]["apc_height_m"] = 15
        data["disturbances"]["all_enabled"] = False
        for group in ("translation", "attitude"):
            for factor in data["disturbances"][group].values():
                factor["enabled"] = False
        app.apply_config(Config.model_validate(data), PROJECT / "main" / "config.yaml")
        assert app.all_accept_button.winfo_exists()
        if args.review:
            from datetime import datetime
            from zoneinfo import ZoneInfo

            source = args.review.resolve()
            destination = PROJECT/"main"/"gui_outputs"/("all_acceptance_review_"+datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f"))
            destination.mkdir()
            shutil.copy2(source/"acceptance.json", destination/"acceptance.json")
            result = json.loads((source/"result.json").read_text(encoding="utf-8"))
            app.active_job = destination
            app.process = object()
            app.started = time.monotonic()
            app.progress_values = [0., 0.]
            app.progress_percent = 0
            app.all_accept_button.configure(state="disabled")
            app.events.put(("event", {"type": "progress", "percent": 100}))
            app.events.put(("event", result))
            app.events.put(("done", 0))
        else:
            app.all_accept_button.invoke()
        assert app.process is not None
        print("JOB " + str(app.active_job), flush=True)
        deadline = time.monotonic()+3600
        while app.process is not None and time.monotonic() < deadline:
            root.update()
            time.sleep(.05)
        assert app.process is None, "All-chain GUI timeout"
        assert app.result and app.result["status"] == "passed", app.log.get("1.0", "end")[-12000:]
        summary = json.loads((app.active_job/"acceptance.json").read_text(encoding="utf-8"))
        assert len(summary["chain_acceptance"]) == 5
        assert all(chain["status"] == "passed" for chain in summary["chain_acceptance"])
        assert len(summary["cross_chain_metrics"]["pairs"]) == 20
        assert len(summary["external_comparison"]["passes"]) == 2
        assert float(app.progress["value"]) == 100
        assert len(app.pose_arrays) == 2 and len(app.track_axes.lines) == 2
        assert app.all_accept_button.instate(["!disabled"])
        record = {"status": "passed", "result": app.result, "chains": 5, "pair_metrics": 20,
                  "progress": float(app.progress["value"]),
                  "note": "UI review replay of existing completed flights" if args.review else "2 s capture, 15 m APC, no A/B injection; full applicable checks"}
        (app.active_job/"gui_integration_check.json").write_text(
            json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(record, ensure_ascii=False), flush=True)
    finally:
        if app.process is not None:
            app.stop()
        root.destroy()


if __name__ == "__main__":
    main()
