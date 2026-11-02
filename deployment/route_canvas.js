"use strict";
(() => {
  const routeCanvas = document.getElementById("route-track");
  if (!routeCanvas) return;
  const records = JSON.parse(document.getElementById("report-data").textContent);
  const platform = document.getElementById("platform");
  const equal = document.getElementById("route-equal");
  const views = [
    {canvas: routeCanvas, hint: document.getElementById("route-hint"), detail: false},
    {canvas: document.getElementById("route-detail-track"),
      hint: document.getElementById("route-detail-hint"), detail: true},
  ].map(view => ({...view, context: view.canvas.getContext("2d"),
    azimuth: -0.7, elevation: 0.45, zoom: 1, drag: null}));

  function drawView(view) {
    const {canvas, context, hint} = view;
    const width = canvas.getBoundingClientRect().width || 900;
    const height = canvas.getBoundingClientRect().height || 430;
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    context.clearRect(0, 0, width, height);
    const item = records[platform.value];
    const route = item?.route_pose || [];
    if (!route.length) {
      hint.textContent = "该平台没有可展示的水平采集段数据。";
      return;
    }
    const points = route.map(row => row.slice(1, 4));
    const lower = [0, 1, 2].map(j => Math.min(...points.map(p => p[j])));
    const upper = [0, 1, 2].map(j => Math.max(...points.map(p => p[j])));
    const physicalSpan = upper.map((value, j) => value - lower[j]);
    const span = physicalSpan.map(value => Math.max(value, 0.01));
    const common = Math.max(...span);
    const center = upper.map((value, j) => (value + lower[j]) / 2);
    const scale = Math.min(width, height) * 0.32 * view.zoom;
    const trueScale = !view.detail && equal.checked;
    const normalize = point => point.map((value, j) =>
      (value - center[j]) / (trueScale ? common : span[j]) * 1.6);
    function project(point) {
      const x = Math.cos(view.azimuth) * point[0] - Math.sin(view.azimuth) * point[1];
      const y = Math.sin(view.azimuth) * point[0] + Math.cos(view.azimuth) * point[1];
      return [width / 2 + scale * x,
        height / 2 - scale * (Math.sin(view.elevation) * y + Math.cos(view.elevation) * point[2])];
    }
    function stroke(a, b, color) {
      context.beginPath();
      context.moveTo(...project(normalize(a)));
      context.lineTo(...project(normalize(b)));
      context.strokeStyle = color;
      context.lineWidth = 1;
      context.stroke();
    }
    const corners = [];
    for (let x = 0; x < 2; x++) for (let y = 0; y < 2; y++) for (let z = 0; z < 2; z++)
      corners.push([x, y, z].map((value, j) => value ? upper[j] : lower[j]));
    for (let index = 0; index < 8; index++) for (let bit = 0; bit < 3; bit++) {
      const other = index ^ (1 << bit);
      if (other > index) stroke(corners[index], corners[other], "#d4e1eb");
    }
    context.beginPath();
    points.forEach((point, index) => {
      const projected = project(normalize(point));
      if (index) context.lineTo(...projected);
      else context.moveTo(...projected);
    });
    context.strokeStyle = view.detail ? "#a65821" : "#236b9b";
    context.lineWidth = 2;
    context.stroke();
    for (const [point, color] of [[points[0], "#168a57"], [points.at(-1), "#b43d4a"]]) {
      context.beginPath();
      context.arc(...project(normalize(point)), 5, 0, 2 * Math.PI);
      context.fillStyle = color;
      context.fill();
    }
    context.font = '13px "Microsoft YaHei",sans-serif';
    context.fillStyle = "#205d9a";
    context.fillText("绿：起点  红：终点", 14, 24);
    const summary = `${item.title} 水平采集段 · ${route.length} 个实际显示采样 · ` +
      `${(route.at(-1)[0] - route[0][0]).toFixed(3)} s · ` +
      `X ${lower[0].toFixed(3)}～${upper[0].toFixed(3)} m，` +
      `Y ${lower[1].toFixed(3)}～${upper[1].toFixed(3)} m，` +
      `Z ${lower[2].toFixed(3)}～${upper[2].toFixed(3)} m`;
    const enlargement = `Y 放大 ${(common / span[1]).toFixed(0)} 倍，` +
      `Z 放大 ${(common / span[2]).toFixed(0)} 倍（相对 X；各轴独立比例）`;
    hint.textContent = `${summary} · ${trueScale ? "真实比例" : enlargement}`;
  }
  function draw() { views.forEach(drawView); }
  function reset() {
    for (const view of views) {
      view.azimuth = -0.7;
      view.elevation = 0.45;
      view.zoom = 1;
    }
    draw();
  }
  platform.addEventListener("change", draw);
  equal.addEventListener("change", draw);
  document.getElementById("route-reset").addEventListener("click", reset);
  for (const view of views) {
    const {canvas} = view;
    canvas.addEventListener("pointerdown", event => {
      view.drag = [event.clientX, event.clientY];
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener("pointermove", event => {
      if (!view.drag) return;
      view.azimuth += (event.clientX - view.drag[0]) * 0.008;
      view.elevation = Math.max(-1.5, Math.min(1.5,
        view.elevation + (event.clientY - view.drag[1]) * 0.008));
      view.drag = [event.clientX, event.clientY];
      drawView(view);
    });
    for (const name of ["pointerup", "pointercancel", "lostpointercapture"])
      canvas.addEventListener(name, () => { view.drag = null; });
    canvas.addEventListener("wheel", event => {
      event.preventDefault();
      view.zoom = Math.max(0.3, Math.min(3, view.zoom * Math.exp(-event.deltaY * 0.001)));
      drawView(view);
    }, {passive: false});
  }
  window.addEventListener("resize", draw);
  draw();
})();
