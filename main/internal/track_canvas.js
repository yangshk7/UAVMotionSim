"use strict";
(() => {
  const canvas = document.getElementById("track-canvas");
  if (!canvas) return;
  const records = JSON.parse(document.getElementById("pose-data").textContent);
  const select = document.getElementById("track-pass");
  const equal = document.getElementById("track-equal");
  const hint = document.getElementById("track-hint");
  const frame = document.getElementById("time");
  const context = canvas.getContext("2d");
  let azimuth = -0.7, elevation = 0.45, zoom = 1, drag = null;

  function draw() {
    const width = canvas.getBoundingClientRect().width || 900;
    const height = canvas.getBoundingClientRect().height || 460;
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, width, height);
    const record = records[select.value];
    if (!record) {
      hint.textContent = "本次没有可显示的 APC 航迹。";
      return;
    }
    const truth = record.navigation_truth.apc_position_sar_m;
    const estimate = record.navigation.apc_position_sar_m;
    if (!truth.length || truth.length !== estimate.length) {
      hint.textContent = "APC 真值与导航采样不完整，请检查原始数据。";
      return;
    }
    const all = truth.concat(estimate);
    const lower = [0, 1, 2].map(j => Math.min(...all.map(p => p[j])));
    const upper = [0, 1, 2].map(j => Math.max(...all.map(p => p[j])));
    const span = upper.map((value, j) => Math.max(value - lower[j], 0.01));
    const common = Math.max(...span);
    const center = upper.map((value, j) => (value + lower[j]) / 2);
    const scale = Math.min(width * 0.38, height * 0.34) * zoom;
    const normalize = p => p.map((value, j) => (value - center[j]) / (equal.checked ? common : span[j]) * 1.6);
    function project(p) {
      const x = Math.cos(azimuth) * p[0] - Math.sin(azimuth) * p[1];
      const y = Math.sin(azimuth) * p[0] + Math.cos(azimuth) * p[1];
      return [width / 2 + scale * x, height / 2 - scale * (Math.sin(elevation) * y + Math.cos(elevation) * p[2])];
    }
    function line(a, b, color, lineWidth) {
      context.beginPath();
      context.moveTo(...project(normalize(a)));
      context.lineTo(...project(normalize(b)));
      context.strokeStyle = color;
      context.lineWidth = lineWidth;
      context.stroke();
    }
    const corners = [];
    for (let x = 0; x < 2; x++) for (let y = 0; y < 2; y++) for (let z = 0; z < 2; z++)
      corners.push([x, y, z].map((v, j) => v ? upper[j] : lower[j]));
    for (let a = 0; a < 8; a++) for (let bit = 0; bit < 3; bit++) {
      const b = a ^ (1 << bit);
      if (b > a) line(corners[a], corners[b], "#d7e2eb", 1);
    }
    function path(points, color) {
      context.beginPath();
      points.forEach((point, index) => {
        const projected = project(normalize(point));
        if (index) context.lineTo(...projected);
        else context.moveTo(...projected);
      });
      context.strokeStyle = color;
      context.lineWidth = 2;
      context.stroke();
    }
    path(truth, "#205d9a");
    path(estimate, "#e37724");
    const current = Math.min(Math.max(Number(frame.value) || 0, 0), truth.length - 1);
    for (const [point, color, radius] of [[truth[0], "#16834a", 5], [truth.at(-1), "#b42f39", 5], [estimate[current], "#e37724", 6]]) {
      context.beginPath();
      context.arc(...project(normalize(point)), radius, 0, 2 * Math.PI);
      context.fillStyle = color;
      context.fill();
    }
    context.font = '13px "Microsoft YaHei",sans-serif';
    context.fillStyle = "#205d9a";
    context.fillText("蓝：APC 真值", 14, 24);
    context.fillStyle = "#c8661b";
    context.fillText("橙：导航估计；圆点为当前采样", 14, 44);
    hint.textContent = `${select.value === "pass1" ? "航次 1" : "航次 2"} · X 前 / Y 右 / Z 上，单位 m · ${equal.checked ? "真实比例" : "各轴独立比例，仅用于观察小偏差"} · X ${lower[0].toFixed(2)}～${upper[0].toFixed(2)}，Y ${lower[1].toFixed(2)}～${upper[1].toFixed(2)}，Z ${lower[2].toFixed(2)}～${upper[2].toFixed(2)}`;
  }
  function reset() { azimuth = -0.7; elevation = 0.45; zoom = 1; draw(); }
  select.addEventListener("change", draw);
  equal.addEventListener("change", draw);
  frame.addEventListener("input", draw);
  document.getElementById("pass").addEventListener("change", () => { select.value = document.getElementById("pass").value; draw(); });
  document.getElementById("track-reset").addEventListener("click", reset);
  canvas.addEventListener("pointerdown", event => { drag = [event.clientX, event.clientY]; canvas.setPointerCapture(event.pointerId); });
  canvas.addEventListener("pointermove", event => {
    if (!drag) return;
    azimuth += (event.clientX - drag[0]) * 0.008;
    elevation = Math.max(-1.5, Math.min(1.5, elevation + (event.clientY - drag[1]) * 0.008));
    drag = [event.clientX, event.clientY];
    draw();
  });
  for (const eventName of ["pointerup", "pointercancel", "lostpointercapture"])
    canvas.addEventListener(eventName, () => { drag = null; });
  canvas.addEventListener("wheel", event => {
    event.preventDefault();
    zoom = Math.max(0.3, Math.min(3, zoom * Math.exp(-event.deltaY * 0.001)));
    draw();
  }, {passive: false});
  window.addEventListener("resize", draw);
  window.refreshTrackCanvas = draw;
  draw();
})();
