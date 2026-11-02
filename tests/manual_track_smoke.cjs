// Exercise the generated offline page with its real two-pass acceptance data.
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');

const html = fs.readFileSync(process.argv[2], 'utf8');
const payload = html.match(/<script id="pose-data" type="application\/json">([\s\S]*?)<\/script>/)[1];
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)];
const trackScript = scripts.at(-1)[1];
const poseScript = scripts[0][1];
const data = JSON.parse(payload);
assert.equal(data.pass1.navigation.apc_position_sar_m.length, 2049);
assert.equal(data.pass2.navigation_truth.apc_position_sar_m.length, 2049);

const trace = [];
const context = new Proxy({}, {get(target, key) {
  if (key in target) return target[key];
  return (...args) => { if (key === 'lineTo') trace.push(args); };
}});
class Element {
  constructor(value='') { this.value=value; this.checked=false; this.listeners={}; this.textContent=''; this.children=[]; }
  addEventListener(name, fn) { this.listeners[name]=fn; }
  appendChild(child) { this.children.push(child); }
  replaceChildren() { this.children=[]; }
  getBoundingClientRect() { return {width:900,height:460}; }
  getContext() { return context; }
  setPointerCapture() {}
}
const ids = Object.fromEntries(['track-canvas','track-pass','track-equal','track-hint','time','pass','track-reset','pose-data','clock','pose','play'].map(id => [id,new Element()]));
ids['track-pass'].value='pass1';
ids['pass'].value='pass1';
ids['time'].value='0';
ids['pose-data'].textContent=payload;
const window = {devicePixelRatio:1,addEventListener() {}};
let tick;
const sandbox = {document:{getElementById:id => ids[id],createElement:()=>new Element()},window,Math,JSON,setInterval:fn=>{tick=fn;return 1;},clearInterval(){}};
vm.createContext(sandbox);
vm.runInContext(poseScript,sandbox,{timeout:5000});
vm.runInContext(trackScript,sandbox,{timeout:5000});
assert.ok(trace.length > 4000);
const before = trace.at(-1);
ids['track-canvas'].listeners.pointerdown({clientX:10,clientY:10,pointerId:1});
ids['track-canvas'].listeners.pointermove({clientX:50,clientY:30});
assert.notDeepEqual(trace.at(-1),before);
ids['track-canvas'].listeners.wheel({deltaY:-100,preventDefault(){}});
ids['track-equal'].checked=true;
ids['track-equal'].listeners.change();
assert.match(ids['track-hint'].textContent,/真实比例/);
ids['track-pass'].value='pass2';
ids['track-pass'].listeners.change();
assert.match(ids['track-hint'].textContent,/航次 2/);
ids['time'].value='100';
ids['time'].listeners.input();
ids['track-reset'].listeners.click();
ids['track-pass'].value='pass1';
ids['track-pass'].listeners.change();
ids['play'].onclick();
const beforePlayback = trace.length;
tick();
assert.ok(trace.length > beforePlayback, 'playback must redraw the 3D point');
console.log('PASS: two real routes, drag, zoom, aspect ratio, pass switch, frame, reset and playback');
