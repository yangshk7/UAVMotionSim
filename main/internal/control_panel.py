"""UAV Motion Sim 桌面上位机。Tk 主线程管理界面，计算在独立进程执行。"""

import argparse
import json
import os
import queue
import threading
import time
import tkinter as tk
import traceback
import webbrowser
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
from control_jobs import create_job, launch_job, parse_event, stop_job
from control_model import (
    FLIGHT_FIELDS,
    LEVER_ARM_FIELDS,
    NAVIGATION_GROUPS,
    RESOURCE_FIELDS,
    Field,
    apply_fields,
    get_value,
)
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from PIL import Image, ImageTk
from trajectory_view import draw_trajectory

from uavsim.config.loader import load_config, save_config
from uavsim.disturbances.presets import NAVIGATION_M
from uavsim.motion.lever_arm import export_apc_from_aligned
from uavsim.runtime.hardware import detect_hardware
from uavsim.visualization.chinese_style import use_chinese_font

PROJECT = Path(__file__).resolve().parents[2]
MAIN = PROJECT / "main"


class ScrollForm(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.canvas = tk.Canvas(self, highlightthickness=0, background="#f1f5f9")
        bar = ttk.Scrollbar(self, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.body = ttk.Frame(self.canvas, padding=14)
        window = self.canvas.create_window(0, 0, anchor="nw", window=self.body)
        self.body.bind("<Configure>", lambda _: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(window, width=e.width))
        self.bind("<Enter>", lambda _: self.canvas.bind_all("<MouseWheel>", self._wheel))
        self.bind("<Leave>", lambda _: self.canvas.unbind_all("<MouseWheel>"))

    def _wheel(self, event):
        self.canvas.yview_scroll(-int(event.delta / 120), "units")


class ControlPanel:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("UAV Motion Sim · 参数控制上位机")
        self.root.geometry("1320x880")
        self.root.minsize(1100, 720)
        self.loading = True
        self.base = load_config(MAIN / "motion_config.yaml")
        self.source = MAIN / "motion_config.yaml"
        self.fields = []
        self.bindings = {}
        self.auto_vectors = {}
        self.vector_entries = {}
        self.events = queue.Queue()
        self.process = None
        self.active_job = None
        self.result = None
        self.cancel_requested = False
        self.started = 0.0
        self.validation_timer = None
        self.image = None
        self.photo = None
        self.pose_arrays = {}
        self.status = tk.StringVar(value="就绪 · 调整参数后可先运行仿真")
        self.config_status = tk.StringVar(value="")
        self.hardware_status = tk.StringVar(value="正在读取 CPU、内存和显存…")
        self.source_text = tk.StringVar(value=str(self.source))
        self.passes = tk.StringVar(value="1")
        self.matlab = tk.StringVar(value="auto")
        self.matlab_exe = tk.StringVar(value="")
        self.sar_project = tk.StringVar(value=str(PROJECT.parent / "L波段仿真"))
        self._style()
        self._build()
        self.loading = False
        self._schedule_validation()
        self.refresh_hardware()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(100, self._poll)

    def _style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", font=("Microsoft YaHei UI", 10))
        style.configure("TFrame", background="#f1f5f9")
        style.configure("TLabel", background="#f1f5f9", foreground="#243c52")
        style.configure("TLabelframe", background="#f1f5f9", bordercolor="#d4dfe8")
        style.configure("TLabelframe.Label", font=("Microsoft YaHei UI", 11, "bold"), foreground="#193954")
        style.configure("TButton", padding=(12, 7))
        style.configure("Primary.TButton", background="#235d8d", foreground="white")
        style.map("Primary.TButton", background=[("active", "#194a73"), ("disabled", "#a2b1bf")])
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 20, "bold"), foreground="#173955")
        style.configure("Hint.TLabel", font=("Microsoft YaHei UI", 9), foreground="#617587")
        style.configure("TNotebook.Tab", padding=(18, 9))
        style.configure("Treeview", rowheight=34)

    def _build(self):
        header = ttk.Frame(self.root, padding=(20, 14, 20, 8))
        header.pack(fill="x")
        ttk.Label(header, text="UAV Motion Sim", style="Title.TLabel").pack(anchor="w")
        ttk.Label(header, text="高频航迹真值与 100 Hz 导航位姿 · 参数控制上位机", style="Hint.TLabel").pack(anchor="w")
        ttk.Label(header, textvariable=self.hardware_status, style="Hint.TLabel", wraplength=1220).pack(
            anchor="w", pady=(7, 0)
        )
        commands = ttk.Frame(self.root, padding=(18, 5))
        commands.pack(fill="x")
        ttk.Button(commands, text="加载配置", command=self.load_dialog).pack(side="left", padx=3)
        ttk.Button(commands, text="另存配置", command=self.save_dialog).pack(side="left", padx=3)
        ttk.Button(commands, text="检查参数", command=self.check_config).pack(side="left", padx=3)
        self.run_button = ttk.Button(
            commands, text="运行仿真", style="Primary.TButton", command=lambda: self.start("simulate")
        )
        self.run_button.pack(side="left", padx=(20, 3))
        self.accept_button = ttk.Button(commands, text="完整验收", command=lambda: self.start("acceptance"))
        self.accept_button.pack(side="left", padx=3)
        self.stop_button = ttk.Button(commands, text="停止当前任务", command=self.stop, state="disabled")
        self.stop_button.pack(side="left", padx=3)
        ttk.Button(commands, text="刷新硬件", command=self.refresh_hardware).pack(side="right", padx=3)
        ttk.Label(self.root, textvariable=self.status, padding=(22, 5)).pack(fill="x")
        self.pages = ttk.Notebook(self.root)
        self.pages.pack(fill="both", expand=True, padx=18, pady=(5, 15))
        parameters = ttk.Frame(self.pages)
        self.monitor = ttk.Frame(self.pages, padding=12)
        self.results = ttk.Frame(self.pages, padding=10)
        self.pages.add(parameters, text="参数配置")
        self.pages.add(self.monitor, text="运行监控")
        self.pages.add(self.results, text="结果查看")
        ttk.Label(parameters, textvariable=self.source_text, style="Hint.TLabel", padding=(12, 6)).pack(
            fill="x"
        )
        notebook = ttk.Notebook(parameters)
        notebook.pack(fill="both", expand=True)
        forms = {}
        for name in ["导航估计误差", "飞行与采样", "运动扰动", "执行与资源"]:
            form = ScrollForm(notebook)
            notebook.add(form, text=name)
            forms[name] = form.body
        self._build_navigation(forms["导航估计误差"])
        group = self._group(forms["飞行与采样"], "飞行基准与多速率采样")
        for field in FLIGHT_FIELDS:
            self._field(group, field)
        antenna_group = self._group(forms["飞行与采样"], "天线相位中心安装 · 仿真杆臂")
        for field in LEVER_ARM_FIELDS:
            self._field(antenna_group, field)
        ttk.Label(
            forms["飞行与采样"],
            text="杆臂从机体质心指向 SAR 天线相位中心：机体 FRD 前/右/下为正，单位 m。默认 (0, 0.10, 0.20) m 来自两份既有仿真规范；属于仿真设定，不是实机测量值。修改后同时作用于真值和导航 APC。",
            style="Hint.TLabel", wraplength=1100,
        ).pack(anchor="w", pady=8)
        ttk.Label(
            forms["飞行与采样"],
            text="按航线长度计算时：总时长=长度÷速度+额外裕量（向上对齐采样网格），适用于默认直线航迹；有效匀速区间需在结果中验收。\n内部 NED/FRD；外部 SAR 为 X 前/Y 右/Z 上。高度以 APC 为准。",
            style="Hint.TLabel",
        ).pack(anchor="w", pady=8)
        self._build_disturbances(forms["运动扰动"])
        self._build_resources(forms["执行与资源"])
        ttk.Label(parameters, textvariable=self.config_status, padding=(12, 9), wraplength=1200).pack(
            fill="x"
        )
        ttk.Label(
            self.monitor,
            text="实时输出 · 当前任务使用启动时的独立参数快照",
            font=("Microsoft YaHei UI", 12, "bold"),
        ).pack(anchor="w")
        self.progress = ttk.Progressbar(self.monitor, mode="indeterminate")
        self.progress.pack(fill="x", pady=10)
        self.log = tk.Text(
            self.monitor,
            wrap="word",
            font=("Consolas", 10),
            background="#142d44",
            foreground="#dceaf4",
            insertbackground="white",
        )
        bar = ttk.Scrollbar(self.monitor, command=self.log.yview)
        self.log.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        self.log.pack(fill="both", expand=True)
        self._build_results()

    def _group(self, parent, title):
        frame = ttk.LabelFrame(parent, text=title, padding=(15, 10))
        frame.pack(fill="x", pady=(0, 14))
        frame.columnconfigure(1, weight=1)
        return frame

    def _field(self, parent, field: Field):
        self.fields.append(field)
        value = get_value(self.base.model_dump(), field.path)
        row = parent.grid_size()[1]
        ttk.Label(parent, text=field.label).grid(row=row, column=0, sticky="w", padx=(0, 20), pady=6)
        holder = ttk.Frame(parent)
        holder.grid(row=row, column=1, sticky="ew", pady=4)
        if field.kind == "bool":
            variable = tk.BooleanVar(value=value)
            ttk.Checkbutton(holder, text="开启", variable=variable).pack(side="left")
            variables = [variable]
        elif field.kind == "vector":
            if value is None:
                nav = self.base.disturbances.navigation_position_error
                value = NAVIGATION_M[nav.level]
            variables = [tk.StringVar(value=str(v)) for v in value]
            entries = []
            labels = (("前 +X", "右 +Y", "下 +Z") if field.path == "antenna.phase_center_frd_m"
                      else ("X / rx", "Y / ry", "Z / rz"))
            for index, var in enumerate(variables):
                ttk.Label(holder, text=labels[index]).pack(
                    side="left", padx=(0 if index == 0 else 15, 4)
                )
                widget = ttk.Entry(holder, textvariable=var, width=12)
                widget.pack(side="left")
                entries.append(widget)
            self.vector_entries[field.path] = entries
            if field.nullable:
                auto = tk.BooleanVar(value=get_value(self.base.model_dump(), field.path) is None)
                self.auto_vectors[field.path] = auto

                def toggle():
                    for entry in entries:
                        entry.configure(state="disabled" if auto.get() else "normal")
                    self._schedule_validation()

                ttk.Checkbutton(holder, text="使用等级预设", variable=auto, command=toggle).pack(
                    side="left", padx=15
                )
                toggle()
        else:
            if field.kind == "seeds":
                value = ", ".join(str(v) for v in value)
            variable = tk.StringVar(value="" if value is None else str(value))
            if field.kind == "choice":
                widget = ttk.Combobox(
                    holder, textvariable=variable, values=field.choices, state="readonly", width=22
                )
            else:
                widget = ttk.Entry(holder, textvariable=variable, width=38 if field.kind == "seeds" else 22)
            widget.pack(side="left", fill="x", expand=field.kind == "seeds")
            if field.nullable:
                ttk.Label(holder, text="留空使用默认/自动值", style="Hint.TLabel").pack(side="left", padx=12)
            variables = [variable]
        self.bindings[field.path] = variables
        for variable in variables:
            variable.trace_add("write", lambda *_: self._schedule_validation())
        return variables

    def _build_navigation(self, parent):
        ttk.Label(
            parent,
            text="独立控制估计误差，不改变真实飞行。σ 为标准差，不是最大误差；相关时间越大，偏差变化越慢。关闭某轴误差需将该轴慢变和白噪声 σ 同时设为 0。",
            wraplength=1100,
            style="Hint.TLabel",
        ).pack(anchor="w", pady=(0, 12))
        for title, name, bias, noise, unit in NAVIGATION_GROUPS:
            prefix = "disturbances." + name
            group = self._group(parent, title)
            for field in [
                Field(prefix + ".enabled", "估计误差开关", "bool"),
                Field(prefix + ".correlation_time_s", "慢变相关时间 (s)"),
                Field(
                    prefix + "." + bias,
                    f"慢变偏差 σ ({unit})",
                    "vector",
                    nullable=name == "navigation_position_error",
                ),
                Field(prefix + "." + noise, f"测量白噪声 σ ({unit})", "vector"),
                Field(prefix + ".random_seed", "独立随机种子", "int", nullable=True),
            ]:
                self._field(group, field)
            if name == "navigation_position_error":
                variables = self._field(
                    group, Field(prefix + ".level", "位置误差等级", "choice", ("small", "medium", "large"))
                )

                def level_changed(*_, var=variables[0], path=prefix + "." + bias):
                    if self.loading or not self.auto_vectors[path].get():
                        return
                    for widget, value in zip(self.bindings[path], NAVIGATION_M[var.get()]):
                        widget.set(str(value))

                variables[0].trace_add("write", level_changed)
        ttk.Label(
            parent,
            text="姿态误差采用 R_est = R_true · Exp(δθ_B)，输入为 FRD 旋转向量标准差（deg）。角速度和加速度目前仍为 truth_ 真值通道。",
            wraplength=1100,
            style="Hint.TLabel",
        ).pack(anchor="w")

    def _build_disturbances(self, parent):
        group = self._group(parent, "总体开关与 OU 时间特性")
        for field in [
            Field("disturbances.all_enabled", "强制全部开启（覆盖各单项开关）", "bool"),
            Field("disturbances.tau_slow_s", "慢 OU 相关时间 (s)"),
            Field("disturbances.tau_fast_s", "快 OU 相关时间 (s)"),
            Field("disturbances.slow_variance_fraction", "慢过程方差权重 (0–1)"),
            Field("disturbances.command_filter_cutoff_hz", "指令平滑截止频率 (Hz)"),
        ]:
            self._field(group, field)
        for group_name, names in [("translation", ("x", "y", "z")), ("attitude", ("roll", "pitch", "yaw"))]:
            for axis, name in enumerate(names):
                prefix = f"disturbances.{group_name}.{name}"
                group = self._group(
                    parent, ("A 平移 / " if group_name == "translation" else "B 姿态 / ") + name.upper()
                )
                mode = self._field(
                    group, Field(prefix + ".injection_mode", "注入层", "choice", ("command", "dynamics"))
                )[0]
                self._field(group, Field(prefix + ".enabled", "该轴扰动开关", "bool"))
                self._field(
                    group, Field(prefix + ".level", "候选强度等级", "choice", ("small", "medium", "large"))
                )
                self._field(
                    group, Field(prefix + ".amplitude", "幅度（OU 为 σ；constant 为常量）", nullable=True)
                )
                unit = Field(prefix + ".unit", "幅度单位", "str")
                self.fields.append(unit)
                unit_var = tk.StringVar()
                self.bindings[unit.path] = [unit_var]
                row = group.grid_size()[1]
                ttk.Label(group, text="幅度单位").grid(row=row, column=0, sticky="w", pady=6)
                ttk.Label(group, textvariable=unit_var).grid(row=row, column=1, sticky="w")
                for field in [
                    Field(prefix + ".profile", "过程类型", "choice", ("ou", "constant")),
                    Field(prefix + ".start_time_s", "开始时刻 (s)"),
                    Field(prefix + ".end_time_s", "结束时刻 (s)", nullable=True),
                    Field(prefix + ".ramp_s", "平滑开关过渡时长 (s)"),
                    Field(prefix + ".random_seed", "独立随机种子", "int", nullable=True),
                ]:
                    self._field(group, field)
                command_unit = ("m/s", "m", "m")[axis] if group_name == "translation" else "deg"
                dynamic_unit = "N" if group_name == "translation" else "N*m"

                def mode_changed(*_, p=prefix, m=mode, unit=unit_var, cu=command_unit, du=dynamic_unit):
                    unit.set(cu if m.get() == "command" else du)
                    if not self.loading:
                        self.bindings[p + ".amplitude"][0].set("")
                        self.status.set(
                            "切换注入层后请重新填写幅度；动力学层必须明确填写 N / N*m，不能套用位置或角度幅值。"
                        )

                mode.trace_add("write", mode_changed)
                mode_changed()

    def _build_resources(self, parent):
        group = self._group(parent, "随机性与资源上限")
        for field in RESOURCE_FIELDS:
            self._field(group, field)
        options = self._group(parent, "运行方式")
        ttk.Label(options, text="仿真航次数").grid(row=0, column=0, sticky="w", pady=7)
        ttk.Combobox(options, textvariable=self.passes, values=["1", "2"], state="readonly", width=12).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(options, text="完整验收固定双航次；仿真模式可选单航次。", style="Hint.TLabel").grid(
            row=1, column=1, sticky="w"
        )
        ttk.Label(options, text="MATLAB 检查").grid(row=2, column=0, sticky="w", pady=7)
        ttk.Combobox(
            options, textvariable=self.matlab, values=["auto", "required", "skip"], state="readonly", width=20
        ).grid(row=2, column=1, sticky="w")
        for row, label, var in [
            (3, "MATLAB 程序路径（可空）", self.matlab_exe),
            (4, "既有 SAR 工程目录", self.sar_project),
        ]:
            ttk.Label(options, text=label).grid(row=row, column=0, sticky="w", pady=7)
            ttk.Entry(options, textvariable=var, width=70).grid(row=row, column=1, sticky="ew")
        ttk.Label(
            parent,
            text="运行仿真：用于参数试验，不执行完整测试/收敛/MATLAB。\n完整验收：运行全部已实现的本地链路。修改采样或采集时长后，MATLAB 接口还需与既有雷达参数相符。\nGPU/显存会检测并记录；当前计算后端为 float64 CPU，不提供未经验证的 GPU 开关。",
            style="Hint.TLabel",
            wraplength=1100,
        ).pack(anchor="w")

    def _build_results(self):
        bar = ttk.Frame(self.results)
        bar.pack(fill="x", pady=(0, 10))
        ttk.Button(bar, text="载入已有结果", command=self.choose_result).pack(side="left", padx=3)
        ttk.Button(bar, text="外部同钟航迹应用杆臂", command=self.apply_external_lever).pack(side="left", padx=3)
        ttk.Button(bar, text="打开离线报告", command=self.open_report).pack(side="left", padx=3)
        ttk.Button(bar, text="打开结果文件夹", command=self.open_folder).pack(side="left", padx=3)
        self.result_label = tk.StringVar(value="尚未载入结果")
        ttk.Label(self.results, textvariable=self.result_label, style="Hint.TLabel", wraplength=1200).pack(
            fill="x"
        )
        tabs = ttk.Notebook(self.results)
        self.results_tabs = tabs
        tabs.pack(fill="both", expand=True, pady=8)
        images = ttk.Frame(tabs, padding=5)
        pose = ttk.Frame(tabs, padding=12)
        track = ttk.Frame(tabs, padding=8)
        tabs.add(images, text="图表预览")
        tabs.add(track, text="三维航迹（可拖动）")
        self.track_tab = track
        tabs.add(pose, text="导航位姿逐帧对照")
        toolbar = ttk.Frame(images)
        toolbar.pack(fill="x")
        self.chart = tk.StringVar()
        self.chart_box = ttk.Combobox(toolbar, textvariable=self.chart, state="readonly", width=65)
        self.chart_box.pack(side="left", padx=5, pady=5)
        self.chart_box.bind("<<ComboboxSelected>>", lambda _: self.show_chart())
        ttk.Button(toolbar, text="适应窗口", command=lambda: self.render_image(True)).pack(side="left")
        ttk.Button(toolbar, text="原始大小", command=lambda: self.render_image(False)).pack(
            side="left", padx=5
        )
        image_frame = ttk.Frame(images)
        image_frame.pack(fill="both", expand=True)
        self.image_canvas = tk.Canvas(image_frame, background="white", highlightthickness=0)
        vbar = ttk.Scrollbar(image_frame, orient="vertical", command=self.image_canvas.yview)
        hbar = ttk.Scrollbar(image_frame, orient="horizontal", command=self.image_canvas.xview)
        self.image_canvas.configure(yscrollcommand=vbar.set, xscrollcommand=hbar.set)
        vbar.pack(side="right", fill="y")
        hbar.pack(side="bottom", fill="x")
        self.image_canvas.pack(fill="both", expand=True)
        self.image_canvas.bind("<Configure>", lambda _: self.render_image(True))
        track_controls = ttk.Frame(track)
        track_controls.pack(fill="x", pady=(0, 5))
        self.track_pass = tk.StringVar(value="pass1")
        self.track_pass_box = ttk.Combobox(
            track_controls, textvariable=self.track_pass, values=[], state="readonly", width=12
        )
        self.track_pass_box.pack(side="left", padx=5)
        self.track_pass_box.bind("<<ComboboxSelected>>", lambda _: self.show_trajectory())
        self.track_point = tk.StringVar(value="cg")
        for text, value in [("质心航迹", "cg"), ("APC 航迹", "apc")]:
            ttk.Radiobutton(
                track_controls,
                text=text,
                variable=self.track_point,
                value=value,
                command=self.show_trajectory,
            ).pack(side="left", padx=7)
        self.track_equal_scale = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            track_controls,
            text="真实比例",
            variable=self.track_equal_scale,
            command=self.show_trajectory,
        ).pack(side="left", padx=10)
        ttk.Button(track_controls, text="重置视角", command=self.reset_trajectory_view).pack(
            side="left", padx=5
        )
        ttk.Label(
            track,
            text="按住左键拖动旋转，右键拖动缩放；默认各轴独立缩放以显示小偏差，勾选“真实比例”查看实际几何比例。",
            style="Hint.TLabel",
        ).pack(anchor="w", padx=6, pady=(0, 5))
        use_chinese_font()
        self.track_figure = Figure(figsize=(10, 6), dpi=100, layout="constrained")
        self.track_axes = self.track_figure.add_subplot(111, projection="3d")
        self.track_axes.mouse_init()
        self.track_canvas = FigureCanvasTkAgg(self.track_figure, master=track)
        self.track_canvas.get_tk_widget().pack(fill="both", expand=True)
        controls = ttk.Frame(pose)
        controls.pack(fill="x")
        self.pose_pass = tk.StringVar(value="pass1")
        self.pose_box = ttk.Combobox(
            controls, textvariable=self.pose_pass, values=[], state="readonly", width=12
        )
        self.pose_box.pack(side="left")
        self.pose_box.bind("<<ComboboxSelected>>", lambda _: self._load_pose_pass())
        self.pose_time = tk.StringVar(value="—")
        ttk.Label(controls, textvariable=self.pose_time).pack(side="right")
        self.pose_scale = ttk.Scale(pose, from_=0, to=1, command=self.draw_pose)
        self.pose_scale.pack(fill="x", pady=15)
        self.pose_table = ttk.Treeview(
            pose, columns=("field", "truth", "navigation"), show="headings", height=8
        )
        for name, title, width in [
            ("field", "字段 / 单位", 240),
            ("truth", "真实值", 410),
            ("navigation", "导航估计值", 410),
        ]:
            self.pose_table.heading(name, text=title)
            self.pose_table.column(name, width=width, stretch=True)
        self.pose_table.pack(fill="x")
        ttk.Label(
            pose,
            text="位置为质心位置，APC 单独显示。SAR 为 X 前/Y 右/Z 上；姿态为 NED/FRD，四元数为 wxyz。\n导航估计仅叠加所选误差；角速度/加速度仍按 truth_ 真值字段输出。",
            style="Hint.TLabel",
            wraplength=1100,
        ).pack(anchor="w", pady=15)

    def collect_config(self):
        values = {}
        for field in self.fields:
            bindings = self.bindings[field.path]
            if field.path in self.auto_vectors and self.auto_vectors[field.path].get():
                values[field.path] = None
            elif field.kind == "vector":
                values[field.path] = [v.get() for v in bindings]
            else:
                values[field.path] = bindings[0].get()
        return apply_fields(self.base, self.fields, values)

    def _schedule_validation(self):
        if self.loading:
            return
        if self.validation_timer:
            self.root.after_cancel(self.validation_timer)
        self.validation_timer = self.root.after(250, self._live_validation)

    def _live_validation(self):
        self.validation_timer = None
        try:
            config = self.collect_config()
            s = config.simulation
            route = config.trajectory
            target = (
                f"目标 {route.route_length_m:g} m · 速度 {route.speed_mps:g}±{route.speed_tolerance_fraction * 100:g}% m/s · "
                f"裕量 {s.duration_margin_s:g} s · "
                if s.duration_mode == "route_length" else "固定时长 · "
            )
            self.config_status.set(
                f"参数有效 · {target}输出 {s.acquisition_duration_s:g} s · 真值 {round(s.acquisition_duration_s * s.dynamics_rate_hz) + 1:,} 点 · 导航 {round(s.acquisition_duration_s * s.navigation_rate_hz) + 1:,} 点 · 杆臂 FRD {tuple(config.antenna.phase_center_frd_m)} m"
            )
        except ValueError as error:
            self.config_status.set("参数需检查：" + str(error).splitlines()[0])

    def check_config(self):
        try:
            self.collect_config()
            self._live_validation()
            messagebox.showinfo(
                "参数检查", "参数校验通过。数值运行和外部 MATLAB 接口仍会执行各自检查。", parent=self.root
            )
        except ValueError as error:
            messagebox.showerror("参数无效", str(error), parent=self.root)

    def apply_config(self, config, source: Path):
        self.loading = True
        self.base = config
        self.source = source
        data = config.model_dump()
        for field in self.fields:
            value = get_value(data, field.path)
            variables = self.bindings[field.path]
            if field.kind == "vector":
                if field.path in self.auto_vectors:
                    auto = self.auto_vectors[field.path]
                    auto.set(value is None)
                    for widget in self.vector_entries[field.path]:
                        widget.configure(state="disabled" if auto.get() else "normal")
                if value is None:
                    value = NAVIGATION_M[config.disturbances.navigation_position_error.level]
                for variable, item in zip(variables, value):
                    variable.set(str(item))
            elif field.kind == "seeds":
                variables[0].set(", ".join(str(v) for v in value))
            else:
                variables[0].set("" if value is None else value)
        # 加载时不清空已有幅度；仅把合法单位显示为明确物理单位。
        for group, names in [("translation", ("x", "y", "z")), ("attitude", ("roll", "pitch", "yaw"))]:
            for axis, name in enumerate(names):
                prefix = f"disturbances.{group}.{name}"
                mode = get_value(data, prefix + ".injection_mode")
                unit = (
                    (("m/s", "m", "m")[axis] if group == "translation" else "deg")
                    if mode == "command"
                    else ("N" if group == "translation" else "N*m")
                )
                self.bindings[prefix + ".unit"][0].set(unit)
        self.source_text.set(str(source) + " · 未显示的参数保留于完整配置")
        self.loading = False
        self._schedule_validation()

    def load_dialog(self):
        path = filedialog.askopenfilename(
            parent=self.root,
            title="加载完整或继承 YAML 配置",
            initialdir=MAIN,
            filetypes=[("YAML", "*.yaml *.yml")],
        )
        if path:
            try:
                self.apply_config(load_config(path), Path(path))
            except Exception as error:
                messagebox.showerror("加载失败", str(error), parent=self.root)

    def save_dialog(self):
        try:
            config = self.collect_config()
            directory = MAIN / "gui_configs"
            directory.mkdir(exist_ok=True)
            path = filedialog.asksaveasfilename(
                parent=self.root,
                title="保存完整参数配置",
                initialdir=directory,
                initialfile="experiment_config.yaml",
                defaultextension=".yaml",
                filetypes=[("YAML", "*.yaml")],
            )
            if path:
                if self.process is not None and Path(path).resolve().is_relative_to(
                    self.active_job.resolve()
                ):
                    raise ValueError("当前任务目录中的参数快照不能覆盖，请另存到其他位置。")
                save_config(config, path)
                self.status.set("配置已保存：" + str(path))
        except Exception as error:
            messagebox.showerror("保存失败", str(error), parent=self.root)

    def start(self, mode: str):
        if self.process is not None:
            return
        try:
            config = self.collect_config()
            passes = 2 if mode == "acceptance" else int(self.passes.get())
            if len(config.pass_seeds) < passes:
                raise ValueError(f"至少需要 {passes} 个独立航次种子")
            data = config.model_dump()
            data["output"].update(save_mat=True, save_csv=True, save_plots=True)
            from uavsim.config.validator import Config

            config = Config.model_validate(data)
            root = create_job(PROJECT, config, mode)
            process = launch_job(
                PROJECT,
                root,
                mode,
                passes,
                self.matlab.get(),
                self.matlab_exe.get().strip(),
                self.sar_project.get().strip(),
            )
        except Exception as error:
            messagebox.showerror("无法启动", str(error), parent=self.root)
            return
        self.process = process
        self.active_job = root
        self.cancel_requested = False
        self.started = time.monotonic()
        self.result = None
        self.run_button.configure(state="disabled")
        self.accept_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.progress.start(12)
        self.status.set(
            "正在"
            + ("运行完整验收" if mode == "acceptance" else "运行参数仿真")
            + " · 参数快照已锁定，后续编辑只影响下一次任务"
        )
        self.log.insert("end", f"\n===== {mode} | {root.name} =====\n")
        self.pages.select(self.monitor)

        def read_output():
            try:
                with (root / "worker.log").open("w", encoding="utf-8") as log:
                    for line in process.stdout:
                        log.write(line)
                        log.flush()
                        try:
                            event = parse_event(line)
                        except (ValueError, json.JSONDecodeError):
                            event = None
                        self.events.put(("event", event) if event else ("log", line))
                process.stdout.close()
                self.events.put(("done", process.wait()))
            except Exception as error:
                self.events.put(("log", "日志读取失败：" + str(error) + "\n"))
                self.events.put(("done", process.wait()))

        threading.Thread(target=read_output, daemon=True).start()

    def stop(self):
        if self.process is None or self.cancel_requested:
            return
        self.cancel_requested = True
        self.stop_button.configure(state="disabled")
        self.status.set("正在终止本次任务树；已生成的部分结果会保留")
        process = self.process

        def stop_background():
            try:
                stop_job(process)
            except Exception as error:
                self.events.put(("stop_error", f"停止任务失败：{error}\n"))

        threading.Thread(target=stop_background, daemon=True).start()

    def _poll(self):
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "log":
                    self.log.insert("end", value)
                    self.log.see("end")
                elif kind == "stop_error":
                    self.cancel_requested = False
                    self.stop_button.configure(state="normal")
                    self.status.set(value.strip())
                    self.log.insert("end", value)
                elif kind == "hardware":
                    self._display_hardware(value)
                elif kind == "event":
                    if value["type"] == "result":
                        self.result = value
                    else:
                        self.log.insert("end", "任务错误：" + value["message"] + "\n")
                elif kind == "done":
                    self.progress.stop()
                    self.run_button.configure(state="normal")
                    self.accept_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
                    self.process = None
                    state = (
                        "已取消"
                        if self.cancel_requested
                        else (
                            "仿真完成（未执行完整验收）"
                            if self.result and self.result["status"] == "simulated"
                            else "完整验收通过"
                            if self.result and self.result["status"] == "passed"
                            else "部分完成"
                            if self.result and self.result["status"] == "partial"
                            else "运行失败，请查看日志"
                        )
                    )
                    self.status.set(
                        f"{state} · 用时 {time.monotonic() - self.started:.1f} s · {self.active_job.name}"
                    )
                    (self.active_job / "session.json").write_text(
                        json.dumps(
                            {"status": state, "returncode": value, "result": self.result},
                            ensure_ascii=False,
                            indent=2,
                        ),
                        encoding="utf-8",
                    )
                    if self.result and not self.cancel_requested:
                        try:
                            self.load_result(Path(self.result["data_directory"]), Path(self.result["report"]))
                            self.pages.select(self.results)
                        except Exception as error:
                            self.status.set("计算已结束，但结果读取失败：" + str(error))
                            self.log.insert("end", traceback.format_exc() + "\n")
        except queue.Empty:
            pass
        self.root.after(100, self._poll)

    def refresh_hardware(self):
        def probe():
            try:
                self.events.put(("hardware", detect_hardware()))
            except Exception as error:
                self.events.put(("log", "硬件检测失败：" + str(error) + "\n"))

        threading.Thread(target=probe, daemon=True).start()

    def _display_hardware(self, data):
        available = data.get("memory_available_bytes")
        memory = f"{available / 1024**3:.1f} GiB 可用内存" if available is not None else "可用内存未知"
        cards = []
        for gpu in data.get("gpus", []):
            free = gpu.get("free_memory_bytes")
            cards.append(gpu["name"] + (f" · 空闲显存 {free / 1024**3:.1f} GiB" if free is not None else ""))
        self.hardware_status.set(
            f"CPU {data.get('logical_cpus', '?')} 线程 · {memory} · "
            + (" / ".join(cards) or "未检测到 GPU")
            + " · 当前 CPU 后端"
        )

    def load_result(self, data_dir: Path, report: Path | None = None):
        if not any(
            (data_dir / f"navigation_{name}.csv").is_file()
            and (data_dir / f"navigation_truth_{name}.csv").is_file()
            for name in ("pass1", "pass2")
        ):
            raise ValueError(
                "所选目录没有完整导航位姿 CSV，请选择包含 navigation_pass1.csv 和对应真值的数据目录。"
            )
        self.data_dir = data_dir
        self.report_path = report
        self.result_label.set(str(data_dir))
        self.image = None
        self.photo = None
        self.image_canvas.delete("all")
        preferred = [
            "navigation_position_comparison.png",
            "navigation_velocity_comparison.png",
            "navigation_attitude_comparison.png",
            "navigation_pose_errors.png",
            "trajectory_3d.png",
            "motion_states.png",
            "antenna_geometry.png",
            "actuator_and_quaternion_checks.png",
        ]
        files = [name for name in preferred if (data_dir / name).exists()]
        files += [path.name for path in sorted(data_dir.glob("*.png")) if path.name not in files]
        self.chart_box.configure(values=files)
        if files:
            self.chart.set(files[0])
            self.show_chart()
        self.pose_arrays = {}
        for name in ["pass1", "pass2"]:
            nav, truth = data_dir / f"navigation_{name}.csv", data_dir / f"navigation_truth_{name}.csv"
            if nav.exists() and truth.exists():
                self.pose_arrays[name] = (
                    np.atleast_1d(np.genfromtxt(nav, delimiter=",", names=True)),
                    np.atleast_1d(np.genfromtxt(truth, delimiter=",", names=True)),
                )
        self.pose_box.configure(values=list(self.pose_arrays))
        self.track_pass_box.configure(values=list(self.pose_arrays))
        if self.pose_arrays:
            self.pose_pass.set(next(iter(self.pose_arrays)))
            self._load_pose_pass()
            self.track_pass.set(next(iter(self.pose_arrays)))
            self.reset_trajectory_view()

    def show_trajectory(self):
        if self.track_pass.get() not in self.pose_arrays:
            self.track_axes.clear()
            self.track_canvas.draw_idle()
            return
        elevation, azimuth = self.track_axes.elev, self.track_axes.azim
        navigation, truth = self.pose_arrays[self.track_pass.get()]
        draw_trajectory(
            self.track_axes, navigation, truth, self.track_point.get(), self.track_equal_scale.get()
        )
        self.track_axes.view_init(elev=elevation, azim=azimuth)
        self.track_canvas.draw_idle()

    def reset_trajectory_view(self):
        self.track_axes.view_init(elev=24, azim=-62)
        self.show_trajectory()

    def choose_result(self):
        directory = filedialog.askdirectory(
            parent=self.root,
            title="选择包含 navigation_pass*.csv 的数据目录",
            initialdir=MAIN / "gui_outputs",
        )
        if directory:
            data = Path(directory)
            report = data / "index.html"
            if not report.exists():
                report = data.parent / "index.html"
            try:
                self.load_result(data, report if report.exists() else None)
            except Exception as error:
                messagebox.showerror("读取结果失败", str(error), parent=self.root)

    def apply_external_lever(self):
        """用当前上位机杆臂生成外部同钟航迹的 APC 派生文件。"""
        try:
            config = self.collect_config()
            directory = filedialog.askdirectory(
                parent=self.root,
                title="选择含 alignment.json 的外部同钟航迹目录",
                initialdir=PROJECT / "results",
            )
            if not directory:
                return
            source = Path(directory)
            destination = source / ("lever_arm_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
            report = export_apc_from_aligned(source, destination, config.antenna.phase_center_frd_m)
            self.status.set(f"已生成仿真 APC：{destination}")
            messagebox.showinfo(
                "仿真杆臂已应用",
                f"FRD 杆臂 {report['lever_frd_m']} m\n真值 {report['samples']['truth']} 点；导航 {report['samples']['navigation']} 点。\n结果：{destination}\n这不是实机杆臂测量标定。",
                parent=self.root,
            )
        except Exception as error:
            messagebox.showerror("应用杆臂失败", str(error), parent=self.root)

    def show_chart(self):
        if not getattr(self, "data_dir", None) or not self.chart.get():
            return
        with Image.open(self.data_dir / self.chart.get()) as image:
            self.image = image.convert("RGB").copy()
        self.render_image(True)

    def render_image(self, fit=True):
        if self.image is None:
            return
        image = self.image.copy()
        if fit:
            image.thumbnail(
                (
                    max(100, self.image_canvas.winfo_width() - 8),
                    max(100, self.image_canvas.winfo_height() - 8),
                ),
                Image.Resampling.LANCZOS,
            )
        self.photo = ImageTk.PhotoImage(image, master=self.root)
        self.image_canvas.delete("all")
        self.image_canvas.create_image(4, 4, anchor="nw", image=self.photo)
        self.image_canvas.configure(scrollregion=(0, 0, image.width + 8, image.height + 8))

    def _load_pose_pass(self):
        if self.pose_pass.get() in self.pose_arrays:
            nav, _ = self.pose_arrays[self.pose_pass.get()]
            self.pose_scale.configure(to=max(1, len(nav) - 1))
            self.pose_scale.set(0)
            self.draw_pose(0)

    def draw_pose(self, value):
        if self.pose_pass.get() not in self.pose_arrays:
            return
        nav, truth = self.pose_arrays[self.pose_pass.get()]
        index = min(len(nav) - 1, max(0, round(float(value))))
        self.pose_time.set(f"{nav['time_s'][index]:.3f} s · {index + 1}/{len(nav)}")
        self.pose_table.delete(*self.pose_table.get_children())
        for title, fields in [
            ("质心位置 SAR / m", ("x_sar_m", "y_sar_m", "z_sar_m")),
            ("速度 SAR / m/s", ("vx_sar_mps", "vy_sar_mps", "vz_sar_mps")),
            ("Roll/Pitch/Yaw / deg", ("roll_ned_deg", "pitch_ned_deg", "yaw_ned_deg")),
            ("四元数 NB / wxyz", ("qw_nb", "qx_nb", "qy_nb", "qz_nb")),
            ("APC 位置 SAR / m", ("apc_x_sar_m", "apc_y_sar_m", "apc_z_sar_m")),
        ]:
            values = [" , ".join(f"{array[field][index]:.6f}" for field in fields) for array in [truth, nav]]
            self.pose_table.insert("", "end", values=(title, *values))

    def open_report(self):
        report = getattr(self, "report_path", None)
        if report and report.exists():
            webbrowser.open(report.resolve().as_uri())
        else:
            messagebox.showinfo("结果尚未就绪", "先运行任务或加载带报告的结果目录。", parent=self.root)

    def open_folder(self):
        directory = getattr(self, "data_dir", None) or self.active_job
        if directory and directory.exists():
            if os.name == "nt":
                os.startfile(str(directory))
            else:
                webbrowser.open(directory.resolve().as_uri())

    def close(self):
        if self.process is not None:
            if not messagebox.askyesno(
                "任务仍在运行", "终止本次任务并关闭上位机？部分结果会保留。", parent=self.root
            ):
                return
            self.stop()

            def wait():
                if self.process is None:
                    self.root.destroy()
                elif not self.cancel_requested:
                    return
                else:
                    self.root.after(100, wait)

            wait()
        else:
            self.root.destroy()


def main():
    try:
        parser = argparse.ArgumentParser(description="UAV Motion Sim desktop control panel")
        parser.add_argument("--result-dir", type=Path, help="启动时载入已有导航结果目录")
        args = parser.parse_args()
        root = tk.Tk()
        app = ControlPanel(root)
        if args.result_dir:
            app.load_result(args.result_dir.resolve())
            app.pages.select(app.results)
            app.results_tabs.select(app.track_tab)
        root.mainloop()
    except Exception:
        directory = MAIN / "gui_logs"
        directory.mkdir(exist_ok=True)
        path = directory / f"startup_error_{time.time_ns()}.log"
        path.write_text(traceback.format_exc(), encoding="utf-8")
        raise


if __name__ == "__main__":
    main()
