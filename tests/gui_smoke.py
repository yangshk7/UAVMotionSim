"""显式运行的 Tk 集成自检；不放入 pytest 自动集合，避免验收子进程递归启动 GUI。"""

import argparse
import json
import sys
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path

from matplotlib.backend_bases import MouseButton, MouseEvent

from uavsim.config.loader import load_config

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "main" / "internal"))


def wait_for_job(root, app, timeout_s):
    deadline = time.monotonic() + timeout_s
    while app.process is not None and time.monotonic() < deadline:
        root.update()
        time.sleep(0.04)
    root.update()
    if app.process is not None:
        app.stop()
        raise TimeoutError("GUI background job timed out")


def main():
    from control_panel import ControlPanel

    parser = argparse.ArgumentParser()
    parser.add_argument("--full", action="store_true")
    args = parser.parse_args()
    root = tk.Tk()
    root.withdraw()
    app = ControlPanel(root)
    root.update()
    report = {"tk_version": tk.TkVersion, "checks": {}}
    try:
        path = "disturbances.navigation_position_error.bias_std_sar_m"
        app.auto_vectors[path].set(False)
        app.bindings[path][0].set(".2")
        app.bindings["disturbances.navigation_velocity_error.bias_std_sar_mps"][0].set(".04")
        app.bindings["disturbances.navigation_attitude_error.bias_std_body_deg"][0].set(".15")
        collected = app.collect_config()
        assert collected.disturbances.navigation_position_error.bias_std_sar_m[0] == 0.2
        assert collected.disturbances.navigation_velocity_error.bias_std_sar_mps[0] == 0.04
        assert collected.disturbances.navigation_attitude_error.bias_std_body_deg[0] == 0.15
        assert collected.vehicle == app.base.vehicle
        report["checks"]["navigation_parameters_and_hidden_values"] = "passed"
        app.bindings["trajectory.speed_mps"][0].set("nan")
        try:
            app.collect_config()
        except ValueError:
            report["checks"]["invalid_input_rejected"] = "passed"
        else:
            raise AssertionError("NaN input accepted")
        app.apply_config(load_config(PROJECT / "main" / "config.yaml"), PROJECT / "main" / "config.yaml")
        app.bindings["disturbances.translation.y.injection_mode"][0].set("dynamics")
        assert app.bindings["disturbances.translation.y.unit"][0].get() == "N"
        assert app.bindings["disturbances.translation.y.amplitude"][0].get() == ""
        try:
            app.collect_config()
        except ValueError:
            report["checks"]["dynamics_amplitude_requires_explicit_unit_value"] = "passed"
        else:
            raise AssertionError("Unspecified dynamics force accepted")
        app.bindings["disturbances.translation.y.amplitude"][0].set("1.5")
        assert app.collect_config().disturbances.translation["y"].amplitude == 1.5
        app.apply_config(load_config(PROJECT / "main" / "config.yaml"), PROJECT / "main" / "config.yaml")
        for key, value in [
            ("simulation.acquisition_duration_s", ".2"),
            ("simulation.pre_stabilization_s", ".1"),
            ("simulation.dynamics_rate_hz", "1600"),
        ]:
            app.bindings[key][0].set(value)
        app.passes.set("1")
        original = app.collect_config().disturbances.navigation_velocity_error.bias_std_sar_mps[0]
        app.start("simulate")
        assert app.process is not None
        first = app.active_job
        app.bindings["disturbances.navigation_velocity_error.bias_std_sar_mps"][0].set(".99")
        root.update()
        assert (
            load_config(first / "input_config.yaml").disturbances.navigation_velocity_error.bias_std_sar_mps[
                0
            ]
            == original
        )
        wait_for_job(root, app, 90)
        assert app.result is not None and app.result["status"] == "simulated", app.log.get("1.0", "end")
        assert len(app.pose_table.get_children()) == 5
        assert app.image is not None and app.photo is not None
        assert app.track_canvas.get_tk_widget().winfo_exists()
        assert len(app.track_axes.lines) == 2
        nav, truth = app.pose_arrays["pass1"]
        assert list(app.track_axes.lines[0].get_data_3d()[0]) == list(truth["x_sar_m"])
        assert list(app.track_axes.lines[1].get_data_3d()[0]) == list(nav["x_sar_m"])
        app.track_point.set("apc")
        app.show_trajectory()
        assert list(app.track_axes.lines[1].get_data_3d()[2]) == list(nav["apc_z_sar_m"])
        app.track_equal_scale.set(True)
        app.show_trajectory()
        app.track_canvas.draw()
        bounds = app.track_axes.bbox
        center_x = round((bounds.x0 + bounds.x1) / 2)
        center_y = round((bounds.y0 + bounds.y1) / 2)
        before_azimuth = app.track_axes.azim
        for event_name, dx, dy in [
            ("button_press_event", 0, 0),
            ("motion_notify_event", 40, 20),
            ("button_release_event", 40, 20),
        ]:
            event = MouseEvent(
                event_name,
                app.track_canvas,
                center_x + dx,
                center_y + dy,
                button=MouseButton.LEFT,
            )
            app.track_canvas.callbacks.process(event_name, event)
        assert app.track_axes.azim != before_azimuth
        app.reset_trajectory_view()
        assert app.track_axes.azim == -62
        report["checks"]["interactive_3d_track_from_navigation_csv"] = "passed"
        app.pose_scale.set(10)
        root.update()
        app.draw_pose(10)
        assert app.pose_time.get().startswith("0.100 s")
        assert (first / "result.json").is_file()
        current_directory = app.data_dir
        try:
            app.load_result(first)
        except ValueError:
            assert app.data_dir == current_directory
            report["checks"]["invalid_history_directory_rejected"] = "passed"
        else:
            raise AssertionError("Invalid history directory changed visible result")
        report["checks"]["single_pass_worker_and_embedded_results"] = "passed"
        report["checks"]["immutable_running_snapshot"] = "passed"
        report["simulation_job"] = str(first)
        app.apply_config(load_config(PROJECT / "main" / "config.yaml"), PROJECT / "main" / "config.yaml")
        app.start("simulate")
        assert app.process is not None
        cancelled = app.active_job
        app.stop()
        wait_for_job(root, app, 30)
        assert app.cancel_requested
        assert "已取消" in app.status.get()
        assert (cancelled / "input_config.yaml").exists()
        assert json.loads((cancelled / "session.json").read_text(encoding="utf-8"))["result"] is None
        report["checks"]["owned_job_cancellation_preserves_artifacts"] = "passed"
        report["cancelled_job"] = str(cancelled)
        if args.full:
            app.apply_config(load_config(PROJECT / "main" / "config.yaml"), PROJECT / "main" / "config.yaml")
            app.matlab.set("required")
            app.start("acceptance")
            assert app.process is not None
            full = app.active_job
            wait_for_job(root, app, 600)
            assert app.result is not None and app.result["status"] == "passed", app.log.get("1.0", "end")
            assert len(app.pose_arrays) == 2
            assert len(app.pose_table.get_children()) == 5
            report["checks"]["full_acceptance_from_gui"] = "passed"
            report["full_job"] = str(full)
            report["full_result"] = app.result
        report["status"] = "passed"
        target = (
            PROJECT / "main" / "gui_outputs" / ("gui_smoke_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
        )
        target.mkdir(parents=True)
        (target / "gui_validation.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        print(target, flush=True)
    finally:
        if app.process is not None:
            from control_jobs import stop_job

            stop_job(app.process)
        root.destroy()


if __name__ == "__main__":
    main()
