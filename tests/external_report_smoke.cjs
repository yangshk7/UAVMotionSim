// Execute the actual offline report script with real data and a small DOM/canvas harness.
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const assert = require('node:assert/strict');
const html = fs.readFileSync(process.argv[2], 'utf8');
for (const match of html.matchAll(/(?:href|src)="([^"]+)"/g)) {
  assert.ok(fs.existsSync(path.resolve(path.dirname(process.argv[2]), decodeURIComponent(match[1]))), match[1]);
}
const payload = html.match(/<script id="report-data" type="application\/json">([\s\S]*?)<\/script>/)[1];
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(match=>match[1]);
let ticks = null;
const trace = [];
const ctx = new Proxy({}, {get(target, key) {
  if (key in target) return target[key];
  return (...args) => { if (key === 'lineTo') trace.push(args); };
}});
class Element {
  constructor(id='') { this.id=id; this.value=''; this.children=[]; this.listeners={}; this.textContent=''; this.checked=false; }
  appendChild(child) { this.children.push(child); if (this.id==='platform' && !this.value) this.value=child.value; }
  replaceChildren() { this.children=[]; }
  addEventListener(name, fn) { const old=this.listeners[name]; this.listeners[name]=old? (...args)=>{old(...args);fn(...args);}:fn; }
  getBoundingClientRect() { return {width:900,height:430}; }
  getContext() { return ctx; }
  setPointerCapture() {}
}
const elements = Object.fromEntries(['report-data','platform','time','track','equal','pose','clock','plot-hint','reset','play','route-track','route-equal','route-hint','route-reset','route-detail-track','route-detail-hint'].map(id=>[id,new Element(id)]));
elements['report-data'].textContent=payload; elements.time.value='0';
elements.equal.checked=/<input id="equal"[^>]*checked/.test(html);
elements['route-equal'].checked=/<input id="route-equal"[^>]*checked/.test(html);
const context = vm.createContext({document:{getElementById:id=>elements[id],createElement:()=>new Element()},
  window:{devicePixelRatio:1,addEventListener(){}},setInterval:fn=>{ticks=fn;return 1;},clearInterval:()=>{ticks=null;},console});
for (const script of scripts) vm.runInContext(script, context);
assert.equal(elements.platform.children.length, 2);
assert.equal(elements.pose.children.length, 4);
elements.time.value='10'; elements.time.listeners.input();
assert.match(elements.clock.textContent, /11\//);
const before=JSON.stringify(trace.at(-1));
elements.track.listeners.pointerdown({clientX:100,clientY:100,pointerId:1});
elements.track.listeners.pointermove({clientX:160,clientY:120});
assert.notEqual(JSON.stringify(trace.at(-1)), before);
elements.track.listeners.pointerup();
elements.track.listeners.wheel({deltaY:-100,preventDefault(){}});
elements.equal.checked=true; elements.equal.listeners.change();
elements.platform.value=elements.platform.children[1].value; elements.platform.listeners.change();
assert.equal(elements.pose.children.length, 4);
elements.play.onclick(); assert.ok(ticks); ticks(); assert.equal(Number(elements.time.value),5);
elements.play.onclick(); assert.equal(ticks,null);
assert.equal(vm.runInContext('nearest([[3,0],[1,0],[2,0]],1.9).row[0]',context),2);
if (html.includes('id="route-track"')) {
  const data=JSON.parse(payload);
  for (const item of Object.values(data)) {
    assert.ok(item.route_pose.length>0);
    assert.ok(Math.min(...item.route_pose.map(row=>row[3]))>280);
  }
  assert.match(elements['route-hint'].textContent,/采集段/);
  assert.match(elements['route-hint'].textContent,/真实比例/);
  assert.match(elements['route-detail-hint'].textContent,/Y 放大 \d+ 倍，Z 放大 \d+ 倍/);
  const magnification=[...elements['route-detail-hint'].textContent.matchAll(/放大 (\d+) 倍/g)].map(match=>Number(match[1]));
  assert.equal(magnification.length,2);
  assert.ok(magnification.every(value=>value>10));
  const beforeRoute=JSON.stringify(trace.at(-1));
  elements['route-track'].listeners.pointerdown({clientX:100,clientY:100,pointerId:2});
  elements['route-track'].listeners.pointermove({clientX:150,clientY:125});
  assert.notEqual(JSON.stringify(trace.at(-1)),beforeRoute);
  elements['route-track'].listeners.wheel({deltaY:-100,preventDefault(){}});
  elements['route-equal'].checked=false; elements['route-equal'].listeners.change();
  assert.match(elements['route-hint'].textContent,/各轴独立比例/);
  const beforeDetail=JSON.stringify(trace.at(-1));
  elements['route-detail-track'].listeners.pointerdown({clientX:100,clientY:100,pointerId:3});
  elements['route-detail-track'].listeners.pointermove({clientX:180,clientY:130});
  assert.notEqual(JSON.stringify(trace.at(-1)),beforeDetail);
  elements['route-detail-track'].listeners.wheel({deltaY:-100,preventDefault(){}});
}
console.log('PASS: real report data, frame slider, platform switch, drag rotation, zoom, aspect mode and playback');
