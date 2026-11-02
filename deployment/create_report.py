"""离线外部平台报告；只显示实际记录，运行清单防止混入旧结果。"""

import argparse
import csv
import html
import json
import math
import os
import re
import webbrowser
from datetime import datetime
from pathlib import Path
from urllib.parse import quote
from zoneinfo import ZoneInfo

PROJECT = Path(__file__).resolve().parents[1]
PLATFORMS = {"sitl": "ArduPilot SITL", "xtdrone": "XTDrone2 / PX4 / Gazebo"}


def runs(base: Path, name: str, route: bool = False) -> list[Path]:
    directory = base / "results" / (name + "_route" if route else name)
    return sorted((p for p in directory.glob("*") if p.is_dir()), key=lambda p: p.name)


def begin(base: Path = PROJECT, route: bool = False) -> Path:
    stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
    root = base / "results" / ("external_route_review" if route else "external_review") / ("review_" + stamp)
    root.mkdir(parents=True)
    manifest = {
        "started_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
        "existing": {name: [p.name for p in runs(base, name, route)] for name in PLATFORMS},
        "route": route,
    }
    (root / "run_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    return root


def read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def display_rows(rows: list, limit: int = 4000) -> list:
    """只为离线图抽取实际采样点；原始 CSV 不修改，不做插值。"""
    if len(rows) <= limit:
        return rows
    stride = math.ceil((len(rows) - 1) / (limit - 1))
    selected = rows[::stride]
    if selected[-1] is not rows[-1]:
        selected.append(rows[-1])
    return selected


def packet(directory: Path, name: str, route: bool = False) -> dict:
    """保留原消息时间；姿态与位置分别采样，不伪造同步真值。"""
    validation = json.loads((directory / "validation.json").read_text(encoding="utf-8"))
    positions = read_csv(directory / "navigation_position_sar.csv")
    attitude_path = directory / (
        "navigation_attitude_ned.csv" if name == "sitl" else "navigation_attitude.csv"
    )
    attitudes = read_csv(attitude_path)
    time_field = "timestamp_s" if positions and "timestamp_s" in positions[0] else "time_boot_s"
    fields = [time_field, "x_sar_m", "y_sar_m", "z_sar_m", "vx_sar_mps", "vy_sar_mps", "vz_sar_mps"]
    pose = [[float(row[key]) for key in fields] for row in display_rows(positions)]
    afields = (
        ["time_boot_s", "roll_rad", "pitch_rad", "yaw_rad"]
        if name == "sitl"
        else [
            "timestamp_s" if attitudes and "timestamp_s" in attitudes[0] else "time_boot_s",
            "qw_nb",
            "qx_nb",
            "qy_nb",
            "qz_nb",
        ]
    )
    attitude = [[float(row[key]) for key in afields] for row in display_rows(attitudes)]
    route_pose = []
    if route:
        route_rows = read_csv(directory / "route_navigation_sar.csv")
        if not route_rows:
            raise ValueError("采集航段 CSV 缺失或为空")
        route_time = "timestamp_s" if "timestamp_s" in route_rows[0] else "time_boot_s"
        route_pose = [
            [float(row[key]) for key in (route_time, "x_sar_m", "y_sar_m", "z_sar_m")]
            for row in display_rows(route_rows)
        ]
    data = {
        "validation": validation,
        "pose": pose,
        "attitude": attitude,
        "route_pose": route_pose,
        "attitude_type": "euler" if name == "sitl" else "quaternion",
        "position_samples": len(positions),
        "attitude_samples": len(attitudes),
        "directory": directory.name,
        "title": PLATFORMS[name],
    }
    json.dumps(data, allow_nan=False)
    return data


def create_report(root: Path, base: Path = PROJECT, codes: dict | None = None, route: bool = False) -> Path:
    manifest_path = root / "run_manifest.json"
    manifest = json.loads(manifest_path.read_text()) if codes is not None else None
    data, stages, cards = {}, [], []

    def link(path: Path, label: str) -> str:
        url = quote(Path(os.path.relpath(path, root)).as_posix(), safe="/.")
        return f'<a href="{url}" target="_blank">{html.escape(label)}</a>'

    for name, title in PLATFORMS.items():
        candidates = runs(base, name, route)
        if manifest:
            candidates = [p for p in candidates if p.name not in manifest["existing"][name]]
        else:
            candidates = candidates[-1:]
        status, detail, directory = "partial", "本次未生成可用记录", None
        if codes and codes[name] not in ("0", "not_run"):
            status, detail = "failed", "入口运行失败，请查看本次组件日志。"
        if len(candidates) == 1:
            directory = candidates[0]
            try:
                entry = packet(directory, name, route)
                data[name] = entry
                passed = entry["validation"].get("passed") is True and bool(entry["pose"])
                if not codes or codes[name] == "0":
                    status = "passed" if passed else "failed"
                detail = f"位置 {entry['position_samples']} 条 · 姿态 {entry['attitude_samples']} 条"
                maximum = entry["validation"].get("max_relative_height_m")
                if isinstance(maximum, (int, float)):
                    detail += f" · 导航 Z 最大 {maximum:.3f} m"
                span = entry["validation"].get("route_x_span_m")
                if route and isinstance(span, (int, float)):
                    detail += f" · 采集段前向位移 {span:.2f} m"
                detail += f" · 场景 {entry['validation'].get('scenario', 'baseline')}"
            except (ValueError, OSError, KeyError) as error:
                status, detail = "failed", "结果读取失败：" + str(error)
        elif len(candidates) > 1:
            detail = "同一运行窗口存在多组新结果，未自动选择；请分别查看原始目录。"
        stages.append(
            {
                "name": name,
                "title": title,
                "status": status,
                "detail": detail,
                "directory": str(directory) if directory else None,
            }
        )
        body = '<p class="hint">本次没有可展示的数据。</p>'
        if directory:
            image = directory / "flight_summary.png"
            if image.exists():
                url = quote(Path(os.path.relpath(image, root)).as_posix(), safe="/.")
                body = (
                    f'<a href="{url}" target="_blank"><img src="{url}" alt="{html.escape(title)} 航迹"></a>'
                )
            else:
                body = '<p class="hint">本次未生成静态图，请查看数据与日志。</p>'
            available = [
                ("validation.json", "验证记录"),
                ("scenario.json", "场景配置"),
                ("scenario_world.sdf", "本次 Gazebo 世界"),
                ("scenario_model.sdf", "本次 Gazebo 机型"),
                ("navigation_position_sar.csv", "位置和速度 CSV"),
                ("navigation_attitude_ned.csv", "姿态 CSV"),
                ("navigation_attitude.csv", "姿态 CSV"),
                ("route_navigation_sar.csv", "采集航段导航 CSV"),
                ("route_target_sar.csv", "采集航段指令 CSV"),
                ("gazebo_truth_enu.csv", "Gazebo 原始物理真值 ENU CSV"),
                ("simulation_truth_wgs84.csv", "SITL 仿真真值 WGS84 CSV"),
                ("simulation_truth_precise_wgs84.csv", "SITL 高精度经纬度与姿态 CSV"),
                ("navigation_truth_comparison.json", "导航与 Gazebo 原始位置差"),
                ("route_aligned_sar.csv", "采集段导航/真值/指令对照 CSV"),
                ("route_truth_aligned_sar.csv", "SITL 采集段导航/仿真真值对照 CSV"),
                ("sitl.log", "SITL 日志"),
                ("px4.log", "PX4 日志"),
                ("gazebo.log", "Gazebo 日志"),
                ("xtdrone.log", "XTDrone2 日志"),
                ("agent.log", "DDS 日志"),
            ]
            body += (
                '<div class="downloads">'
                + "".join(
                    link(directory / file, label) for file, label in available if (directory / file).is_file()
                )
                + "</div>"
            )
        cards.append(
            f'<section class="panel"><h2>{html.escape(title)}</h2><span class="badge {status}">'
            + {"passed": "通过", "failed": "失败", "partial": "未完成"}[status]
            + f'</span><p>{html.escape(detail)}</p><p class="hint">'
            + html.escape(directory.name if directory else "未执行")
            + f"</p>{body}</section>"
        )
    overall = (
        "failed"
        if any(s["status"] == "failed" for s in stages)
        else "passed"
        if all(s["status"] == "passed" for s in stages)
        else "partial"
    )
    mode = ("完整航段 · " if route else "") + ("本次入口运行" if codes is not None else "已保存结果汇总（各平台最新记录）")
    summary = {
        "status": overall,
        "selection": mode,
        "stages": stages,
        "exit_codes": codes,
        "generated_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(),
    }
    template = (Path(__file__).parent / "external_report.html").read_text(encoding="utf-8")
    payload = json.dumps(data, ensure_ascii=False, allow_nan=False).replace("<", "\\u003c")
    substitutions = {
        "__TITLE__": "外部平台 · 完整采集航段验收" if route else "外部平台 · 航迹与起降验收",
        "__SCOPE_NOTE__": (
            "本页检查 300 m 高度、10 m/s 指令、20.48 s 采集航段和落地停桨。位置仍是飞控导航估计；未经物理真值对齐，不声称导航精度或 800 Hz SAR 真值达标。"
            if route else "此页仅汇总外部飞控的短时起降记录；完整 SAR 航线和定位精度未由这些检查确认。"
        ),
        "__MODE__": mode,
        "__FULL_CHECKED__": "checked" if route else "",
        "__FULL_SCALE_NOTE__": (
            "完整起降航迹默认采用真实米制比例。各轴独立比例会把不足 1 m 的横向偏移与约 300 m 的高度变化拉伸到相近画幅；取消勾选仅用于观察微小偏移。"
            if route else "默认各轴独立比例以显示小幅横向偏移；勾选“真实比例”后各方向使用同一米制比例。"
        ),
        "__ROUTE_PANEL__": (
            '<section class="panel" id="horizontal"><h2>仅水平采集段 · 可拖动三维航迹</h2>'
            '<div class="controls"><label><input id="route-equal" type="checkbox" checked> 真实比例</label>'
            '<button id="route-reset">重置水平航段视角</button></div>'
            '<p class="hint">两图仅使用 route_navigation_sar.csv 中的实际采集采样；不含爬升、助跑或降落。'
            '水平段指令是直线飞行，真实比例下小幅偏移会接近不可见。</p>'
            '<h3>航迹（默认真实米制比例）</h3><canvas id="route-track" aria-label="水平采集段三维航迹"></canvas>'
            '<p id="route-hint" class="hint"></p>'
            '<h3>横向与高度偏差细节（显示比例放大）</h3>'
            '<p class="hint">下图仍是同一批实际采样点；Y、Z 按各自变化范围放大，仅用于观察细节，不能按图形外观判断真实横摆幅度。'
            '具体放大倍数与原始米数写在图下。两幅图都可分别拖动和缩放。</p>'
            '<canvas id="route-detail-track" aria-label="水平采集段偏差放大三维航迹"></canvas>'
            '<p id="route-detail-hint" class="hint"></p></section>'
            if route else ""
        ),
        "__STATUS__": overall,
        "__STATUS_TEXT__": {
            "passed": "两个采集航段与落地检查通过" if route else "两个已记录的起降检查通过",
            "failed": "存在失败项，请查看日志",
            "partial": "流程部分完成，缺失项未视为通过",
        }[overall],
        "__GENERATED__": summary["generated_at"],
        "__CARDS__": "".join(cards),
        "__PAYLOAD__": payload,
        "__SCRIPT__": (Path(__file__).parent / "external_report.js").read_text(encoding="utf-8"),
        "__ROUTE_SCRIPT__": (
            (Path(__file__).parent / "route_canvas.js").read_text(encoding="utf-8") if route else ""
        ),
    }
    template = re.sub(
        r"__(?:TITLE|SCOPE_NOTE|MODE|FULL_CHECKED|FULL_SCALE_NOTE|ROUTE_PANEL|ROUTE_SCRIPT|STATUS|STATUS_TEXT|GENERATED|CARDS|PAYLOAD|SCRIPT)__",
        lambda match: substitutions[match[0]],
        template,
    )
    path = root / "index.html"
    path.write_text(template, encoding="utf-8")
    (root / "report_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    pointer = "latest_external_route_report.txt" if route else "latest_external_report.txt"
    pointer_path = base / "main" / "outputs" / pointer
    pointer_path.parent.mkdir(parents=True, exist_ok=True)
    pointer_path.write_text(str(path), encoding="utf-8")
    return path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--begin", action="store_true")
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--sitl-code", default="not_run")
    parser.add_argument("--xtdrone-code", default="not_run")
    parser.add_argument("--open", action="store_true")
    parser.add_argument("--route", action="store_true")
    args = parser.parse_args()
    root = args.run_dir.resolve() if args.run_dir else begin(route=args.route)
    if args.begin:
        print(root.relative_to(PROJECT).as_posix())  # ASCII only: consumed by cmd.exe
        return 0
    codes = {"sitl": args.sitl_code, "xtdrone": args.xtdrone_code} if args.run_dir else None
    path = create_report(root, codes=codes, route=args.route)
    print(path.relative_to(PROJECT).as_posix())
    if args.open:
        webbrowser.open(path.as_uri())
    return (
        0
        if json.loads((root / "report_summary.json").read_text(encoding="utf-8"))["status"] == "passed"
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
