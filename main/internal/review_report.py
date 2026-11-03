"""离线人工验收页面；不依赖外网、Web 服务或额外可视化库。"""

import html
import json
from pathlib import Path

import numpy as np


def acceptance_status(stages: list[dict]) -> str:
    """失败为 failed；未执行/待执行均为 partial，不能冒充全部通过。"""
    if any(stage["status"] == "failed" for stage in stages):
        return "failed"
    if not stages or any(stage["status"] != "passed" for stage in stages):
        return "partial"
    return "passed"


def cursor_payload(result: dict) -> dict:
    """100 Hz 逐帧查看数据；JSON 显示值舍入，原始 MAT/CSV 保留全部精度。"""
    output = {}
    fields = ["position_sar_m", "velocity_sar_mps", "euler_NB_rpy_deg", "q_NB_wxyz", "apc_position_sar_m"]
    for name in ["pass1", "pass2"]:
        if name not in result:
            continue
        output[name] = {"time_s": np.asarray(result[name]["navigation"]["time_s"]).tolist()}
        for source in ["navigation", "navigation_truth"]:
            output[name][source] = {
                field: np.asarray(result[name][source][field]).round(10).tolist() for field in fields
            }
    return output


def create_report(root: Path, summary: dict, result: dict | None = None) -> Path:
    """生成可离线双击打开的验收页；缺失产物只显示未生成，不伪造图表。"""

    def escape(value) -> str:
        return html.escape(str(value), quote=True)

    status = acceptance_status(summary["stages"])
    status_text = {
        "passed": "本次流程全部通过",
        "partial": "流程部分完成，请检查未执行项",
        "failed": "本次流程存在失败项",
    }[status]
    data_dir = Path(summary["data_directory"]) if summary.get("data_directory") else None
    relative = data_dir.relative_to(root).as_posix() if data_dir else ""
    images = [
        (
            "双航次 APC 三维航迹",
            "trajectory_3d.png",
            "各坐标轴独立缩放以显示小幅横向和高度偏差，非等比例机动示意。",
        ),
        (
            "导航位置：真值与估计",
            "navigation_position_comparison.png",
            "逐轴比较质心位置；外部 SAR 坐标，单位 m。",
        ),
        ("导航速度：真值与估计", "navigation_velocity_comparison.png", "三轴速度估计及其真值，单位 m/s。"),
        (
            "导航姿态：真值与估计",
            "navigation_attitude_comparison.png",
            "Roll/Pitch/Yaw，内部 NED/FRD 约定，单位 deg。",
        ),
        ("导航估计误差", "navigation_pose_errors.png", "位置、速度和 SO(3) 旋转向量误差；不是欧拉角差。"),
        ("IMU 与 GNSS 融合诊断", "imu_gnss_diagnostics.png", "加计与陀螺残余偏置、位置预测标准差；不是原始IMU测量日志。"),
        ("运动状态", "motion_states.png", "800 Hz 真实速度、加速度、姿态与体轴角速度。"),
        ("APC 与名义航迹差异", "position_errors.png", "物理运动差异与导航位置误差分开展示。"),
        ("天线杆臂与波束", "antenna_geometry.png", "APC/CG 相对位置、SAR 波束分量、下视角和斜视角。"),
        (
            "旋翼约束与四元数",
            "actuator_and_quaternion_checks.png",
            "单旋翼推力边界、四元数模误差与分配残差。",
        ),
        ("A/B 扰动输入", "disturbance_inputs.png", "指令层与动力学层输入带明确单位。"),
        ("OU 随机过程统计", "ou_statistics.png", "20,000 s 长序列自相关及 PSD，非单次孔径统计标定。"),
        ("1600/3200/6400 Hz 收敛", "numerical_convergence.png", "各积分频率使用同一连续扰动输入。"),
    ]
    cards = []
    for title, filename, description in images:
        image_path = data_dir / filename if data_dir else None
        if image_path is not None and image_path.exists():
            body = f'<a href="{escape(relative)}/{filename}" target="_blank"><img loading="lazy" src="{escape(relative)}/{filename}" alt="{escape(title)}"></a>'
        else:
            body = '<div class="missing">本次未生成该图，请查看流程状态与日志。</div>'
        if filename == "trajectory_3d.png" and result:
            body = (
                '<div class="controls"><label>航次 <select id="track-pass"><option value="pass1">航次 1</option><option value="pass2">航次 2</option></select></label>'
                '<label><input id="track-equal" type="checkbox"> 真实比例</label><button id="track-reset">重置视角</button></div>'
                '<canvas id="track-canvas" aria-label="可拖动的双航次 APC 三维航迹"></canvas>'
                '<p class="hint" id="track-hint">拖动旋转，滚轮缩放。蓝色为 APC 真值，橙色为导航估计。</p>'
                + (f'<p class="hint"><a href="{escape(relative)}/{filename}" target="_blank">打开原始静态图</a></p>' if image_path is not None and image_path.exists() else '')
            )
        cards.append(
            f'<article class="figure"><h3>{escape(title)}</h3><p>{escape(description)}</p>{body}</article>'
        )
    stages = "".join(
        f'<tr><td>{escape(item["name"])}</td><td><span class="badge {escape(item["status"])}">{escape({"passed": "通过", "failed": "失败", "skipped": "未执行"}.get(item["status"], "待完成"))}</span></td><td>{escape(item.get("detail", ""))}</td></tr>'
        for item in summary["stages"]
    )
    files = []
    if data_dir:
        for filename, label in [
            ("motion_truth_pass1.csv", "航段 1 高频物理真值 CSV"),
            ("motion_navigation_pass1.csv", "航段 1 导航估计 CSV"),
            ("motion_segment_pass1.json", "航段 1 数据接口说明 JSON"),
            ("motion_truth_pass2.csv", "航段 2 高频物理真值 CSV"),
            ("motion_navigation_pass2.csv", "航段 2 导航估计 CSV"),
            ("motion_segment_pass2.json", "航段 2 数据接口说明 JSON"),
            ("navigation_pass1.csv", "航次 1 导航位姿 CSV"),
            ("navigation_pass2.csv", "航次 2 导航位姿 CSV"),
            ("navigation_pass1.mat", "航次 1 导航位姿 MAT"),
            ("navigation_pass2.mat", "航次 2 导航位姿 MAT"),
            ("navigation_truth_pass1.csv", "航次 1 位姿真值 CSV"),
            ("navigation_truth_pass2.csv", "航次 2 位姿真值 CSV"),
            ("uav_motion_pass1.mat", "航次 1 完整运动数据"),
            ("uav_motion_pass2.mat", "航次 2 完整运动数据"),
            ("config_snapshot.yaml", "最终配置快照"),
            ("metadata.json", "随机种子与资源决策"),
            ("pytest_results.xml", "自动化测试结果"),
            ("convergence_validation.json", "数值收敛结果"),
            ("matlab_validation.json", "MATLAB 接口结果"),
            ("pytest_output.txt", "测试日志"),
        ]:
            if (data_dir / filename).exists():
                files.append(f'<a class="download" href="{escape(relative)}/{filename}">{escape(label)}</a>')
    payload = json.dumps(cursor_payload(result or {}), ensure_ascii=False, allow_nan=False).replace(
        "<", "\\u003c"
    )
    metrics = summary.get("metrics", {})
    meta = summary.get("resources", {})
    data_cards = [
        ("SAR 脉冲 / 航次", metrics.get("sar_samples", "—")),
        ("导航记录 / 航次", metrics.get("navigation_samples", "—")),
        ("3200→6400 斜距 RMS", metrics.get("range_rms_display", "—")),
        ("自动化测试", metrics.get("tests_display", "—")),
        ("仿真耗时", metrics.get("simulation_display", "—")),
        ("计算后端", f"{meta.get('backend', '—')} · {meta.get('workers', '—')} workers"),
    ]
    kpis = "".join(
        f'<div class="kpi"><span>{escape(label)}</span><b>{escape(value)}</b></div>'
        for label, value in data_cards
    )
    page = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>UAV Motion Sim · 人工验收</title><style>
:root{color-scheme:light;--ink:#193047;--muted:#617286;--border:#dce4ec;--blue:#205d9a}*{box-sizing:border-box}body{margin:0;background:#f1f5f9;color:var(--ink);font-family:"Microsoft YaHei",system-ui,sans-serif;font-size:15px;line-height:1.65}header{background:#142e49;color:#fff;padding:32px max(24px,calc((100vw - 1320px)/2))}header small{color:#b6cddd;letter-spacing:2px}h1{font-size:30px;margin:8px 0}header p{margin:5px 0;color:#d8e5ef}.wrap{max-width:1368px;margin:auto;padding:24px}.status{border-left:5px solid #2b8863;background:#eaf5ef;padding:14px 18px;border-radius:7px;margin-bottom:20px}.status.partial{border-color:#bb8123;background:#fff5df}.status.failed{border-color:#b84747;background:#fff0f0}.kpis{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-bottom:22px}.kpi{background:white;border:1px solid var(--border);border-radius:10px;padding:16px}.kpi span{display:block;color:var(--muted);font-size:13px}.kpi b{font-size:24px;font-weight:650}.panel,.figure{background:white;border:1px solid var(--border);border-radius:10px;padding:20px;margin:0 0 20px}h2{margin:0 0 14px;font-size:21px}h3{margin:0 0 6px;font-size:18px}.figure p,.hint{color:var(--muted);font-size:13px}.figure img{width:100%;height:auto;display:block;border-radius:5px}.missing{padding:38px;background:#f4f6f8;color:var(--muted)}table{border-collapse:collapse;width:100%}td,th{text-align:left;border-bottom:1px solid var(--border);padding:11px 10px;vertical-align:top}th{font-weight:600;color:var(--muted)}.badge{font-size:12px;padding:3px 8px;border-radius:20px;background:#e4f3e9;color:#256d48}.badge.failed{color:#a72e35;background:#ffeaec}.badge.skipped{color:#986315;background:#fff2d5}.controls{display:flex;gap:15px;align-items:center;flex-wrap:wrap;margin-bottom:14px}button,select{font:inherit;border:1px solid var(--border);border-radius:6px;padding:7px 12px;background:white;color:var(--ink)}button{cursor:pointer}input[type=range]{flex:1;min-width:200px;accent-color:var(--blue)}.cursor td{font-family:ui-monospace,Consolas,monospace;white-space:normal;overflow-wrap:anywhere}.downloads{display:flex;flex-wrap:wrap;gap:9px}.download{border:1px solid var(--border);border-radius:6px;padding:8px 12px;color:var(--blue);text-decoration:none}.download:hover{background:#eef5fc}.jump{position:sticky;top:0;background:#f1f5f9eb;backdrop-filter:blur(8px);padding:12px 0;display:flex;gap:10px;z-index:2}.jump a{color:var(--blue);text-decoration:none;padding:4px 10px;background:white;border:1px solid var(--border);border-radius:6px}.foot{font-size:13px;color:var(--muted)}@media(max-width:750px){.wrap{padding:12px}.kpis{grid-template-columns:repeat(2,1fr)}.kpi b{font-size:19px}header{padding:24px}table{font-size:12px}td,th{padding:7px 4px}}#track-canvas{display:block;width:100%;height:460px;border:1px solid var(--border);border-radius:8px;background:#fbfdff;cursor:grab;touch-action:none}#track-canvas:active{cursor:grabbing}
</style></head><body><header><small>UAV MOTION SIM / MANUAL ACCEPTANCE</small><h1>导航位姿与双航次 · 人工验收</h1><p>完整仿真 → 自动化测试 → OU 统计 → 数值收敛 → MATLAB 接口 → 关键产出可视化</p><p>__RUN_NAME__</p></header><main class="wrap">
<div class="status __STATUS__"><b>__STATUS_TEXT__</b><div>__MESSAGE__</div></div>
<div class="kpis">__KPIS__</div><nav class="jump"><a href="#inspection">逐帧查看</a><a href="#checks">验收结果</a><a href="#figures">结果图表</a><a href="#files">导出数据</a></nav>
<section class="panel" id="inspection"><h2>100 Hz 导航位姿 · 逐帧对照</h2><p class="hint">拖动时间轴查看同一时刻的导航估计与真实位姿。此处仅显示已生成的导航采样点；姿态角约定为 NED/FRD，APC 与质心位置明确区分。</p><div class="controls"><select id="pass"><option value="pass1">航次 1</option><option value="pass2">航次 2</option></select><button id="play">播放</button><input id="time" type="range" min="0" max="0" value="0"><b id="clock">—</b></div><table class="cursor"><thead><tr><th>字段 / 单位</th><th>真实值</th><th>导航估计值</th></tr></thead><tbody id="pose"></tbody></table><p class="hint">角速度与加速度的 truth_ 字段保留物理真值；导航误差模型与参数见本次配置中的 imu_gnss。导航姿态误差为 SO(3) 旋转向量，不能将欧拉角差直接当作该误差。</p></section>
<section class="panel" id="checks"><h2>本次运行的验收记录</h2><table><thead><tr><th>项目</th><th>状态</th><th>实际结果</th></tr></thead><tbody>__STAGES__</tbody></table></section>
<section id="figures"><h2>关键产出可视化</h2>__FIGURES__</section>
<section class="panel" id="files"><h2>数据与运行记录</h2><div class="downloads">__FILES__<a class="download" href="acceptance.json">验收汇总 JSON</a><a class="download" href="run.log">完整运行日志</a></div></section>
<section class="panel foot"><b>适用范围</b><p>所有图表来自本次实际运行。机体与导航误差参数为可配置工程初值，尚未完成实机标定或 ArduPilot/Simscape 交叉验证。原 SAR 回波的固定波束权重尚未消费随姿态变化波束。检测 GPU/显存不代表已启用 GPU 动力学后端。</p><p>每次验收保存独立目录、配置、随机种子、日志和数据；自动化通过不能代替人工对运动合理性、坐标约定和实验参数的确认。</p></section></main>
<script id="pose-data" type="application/json">__PAYLOAD__</script><script>
const data=JSON.parse(document.getElementById('pose-data').textContent),pass=document.getElementById('pass'),slider=document.getElementById('time'),clock=document.getElementById('clock'),tbody=document.getElementById('pose');let timer=null;
const fields=[['质心位置 SAR / m','position_sar_m'],['速度 SAR / m/s','velocity_sar_mps'],['Roll / Pitch / Yaw / deg','euler_NB_rpy_deg'],['四元数 NB / wxyz','q_NB_wxyz'],['APC 位置 SAR / m','apc_position_sar_m']];
function draw(){const d=data[pass.value];if(!d){clock.textContent='无本次数据';tbody.textContent='本次流程尚未生成导航数据，请查看验收记录。';return;}slider.max=d.time_s.length-1;const k=Math.min(Number(slider.value),d.time_s.length-1);clock.textContent=d.time_s[k].toFixed(2)+' s · '+k+'/'+(d.time_s.length-1);tbody.replaceChildren();for(const [label,key]of fields){const row=document.createElement('tr');for(const value of [label,d.navigation_truth[key][k].map(x=>x.toFixed(key==='q_NB_wxyz'?7:5)).join(' , '),d.navigation[key][k].map(x=>x.toFixed(key==='q_NB_wxyz'?7:5)).join(' , ')]){const td=document.createElement('td');td.textContent=value;row.appendChild(td);}tbody.appendChild(row);}}
slider.addEventListener('input',draw);pass.addEventListener('change',()=>{slider.value=0;draw();});document.getElementById('play').onclick=function(){if(timer){clearInterval(timer);timer=null;this.textContent='播放';return;}this.textContent='暂停';timer=setInterval(()=>{if(Number(slider.value)>=Number(slider.max)){slider.value=0;}else{slider.value=Math.min(Number(slider.value)+5,Number(slider.max));}draw();window.refreshTrackCanvas?.();},50);};draw();
</script><script>__TRACK_SCRIPT__</script></body></html>"""
    substitutions = {
        "__STATUS__": status,
        "__STATUS_TEXT__": escape(status_text),
        "__RUN_NAME__": escape(root.name),
        "__MESSAGE__": escape(summary.get("message", "请结合图表、配置和原始数据进行人工验收。")),
        "__KPIS__": kpis,
        "__STAGES__": stages,
        "__FIGURES__": "".join(cards),
        "__FILES__": "".join(files),
        "__PAYLOAD__": payload,
        "__TRACK_SCRIPT__": (Path(__file__).parent / "track_canvas.js").read_text(encoding="utf-8"),
    }
    for key, value in substitutions.items():
        page = page.replace(key, value)
    path = root / "index.html"
    path.write_text(page, encoding="utf-8")
    return path
