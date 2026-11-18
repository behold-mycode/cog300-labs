// Unit-test browser event wiring without serial hardware or a DOM dependency.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const events = {}, nodes = {}, requests = [];
function element(id) {
  return nodes[id] ||= {value:'', checked:false, disabled:false, textContent:'', open:false,
    dataset:{}, handlers:{}, classList:{toggle(){}},
    addEventListener(name, fn){this.handlers[name]=fn;}, setAttribute(){},
    querySelector(){return element(id+'button');}, replaceChildren(){}, append(){},
    focus(){}, click(){}, closest(){return null;}};
}
element('mode').value='RACE';
let state = {connected:true,ready:true,status:'Connected',settings:{power:255,minimum:255,invertA:true,invertB:false,followCm:20,wallCm:10,lineTapeLevel:-1},navigation:{encoders:1,odometry:1,geometry:1,localized:1},telemetry:{tapeLevel:1,race:{configured:1,saved:1,speed:50,offset:100,angle:1.8,stage:0},dynamics:true,wall:{sonar:1,calibrated:1,localized:1,leftRange:15,frontWall:.1,rightWall:.9,leftWall:.9,targetL:50,targetR:50,speedL:49,speedR:51}},log:[]};
const document = {getElementById:element, querySelectorAll:()=>[], createElement:()=>element('new'),
  hasFocus:()=>true, hidden:false, addEventListener:(name,fn)=>events['doc:'+name]=fn};
const context = vm.createContext({document,window:{addEventListener:(name,fn)=>events[name]=fn},
  location:{hash:'#token=test'},URLSearchParams,URL,Blob,console,
  setTimeout:()=>1,clearTimeout(){},setInterval(){},AbortController,
  fetch:async (url,options)=>{
    const body=options.body?JSON.parse(options.body):null;requests.push({url,body});
    return {ok:true,json:async()=> url.endsWith('/ports')?[]:url.endsWith('/connect')?{owner:'owner'}:state};
  }});
vm.runInContext(fs.readFileSync(path.join(__dirname,'app.js'),'utf8'),context);
const flush=()=>new Promise(resolve=>setImmediate(resolve));
const key=(code,repeat=false,target=element('drive'))=>({code,repeat,target,preventDefault(){}});
const frames=()=>requests.filter(r=>r.url.endsWith('/frame'));
(async()=>{
  await flush(); await element('connect').handlers.click(); await flush();
  events.keydown(key('ArrowUp'));await flush();assert.deepEqual(frames().at(-1).body.keys,['Up']);
  events.keydown(key('KeyA'));await flush();assert.deepEqual(frames().at(-1).body.keys,['Up','Left']);
  events.keyup(key('ArrowUp'));events.keyup(key('KeyA'));await flush();assert.deepEqual(frames().at(-1).body.keys,[]);
  element('mode').value='RACE';await element('mode').handlers.change();await flush();
  assert.equal(frames().at(-1).body.mode,'MANUAL'); // Selection never starts motion.
  await element('start').handlers.click();await flush();assert.equal(frames().at(-1).body.mode,'RACE');
  element('mode').value='OBJECT';await element('mode').handlers.change();await flush();
  assert.equal(frames().at(-1).body.mode,'MANUAL');
  await element('start').handlers.click();await flush();assert.equal(frames().at(-1).body.mode,'OBJECT');
  element('mode').value='MANUAL';await element('mode').handlers.change();await flush();
  assert.equal(frames().at(-1).body.mode,'MANUAL');
  await element('start').handlers.click();await flush();assert.equal(frames().at(-1).body.mode,'MANUAL');
  events.keydown(key('Space'));await flush();assert.equal(frames().at(-1).body.mode,'MANUAL');
  events.keydown(key('ArrowUp'));await flush();events.blur();await flush();
  events.keydown(key('ArrowUp',true));await flush();assert.deepEqual(frames().at(-1).body.keys,[]);
  const input={closest:()=>true};events.keydown(key('ArrowUp',false,input));await flush();assert.deepEqual(frames().at(-1).body.keys,[]);
  events.keyup(key('ArrowUp'));events.keydown(key('KeyW'));await flush();assert.deepEqual(frames().at(-1).body.keys,['Up']);
  document.hidden=true;events['doc:visibilitychange']();await flush();assert.deepEqual(frames().at(-1).body.keys,[]);
  assert.ok(frames().every((r,i,a)=>i===0||r.body.seq>a[i-1].body.seq));
  assert.ok(!requests.some(r=>r.url.endsWith('/settings')||r.url.endsWith('/navigation')));
  state.telemetry.lineControl={speedL:200,speedR:210,saturated:0};
  state.telemetry.left=240;state.telemetry.motorRight=250;
  await element('stop').handlers.click();await flush();
  assert.match(element('wheelStatus').textContent,/PWM L 240 · R 250/);
  const html=fs.readFileSync(path.join(__dirname,'index.html'),'utf8');
  assert.deepEqual([...html.matchAll(/<option value="([^"]+)"/g)].map(m=>m[1]),['LINE','STOP_ON_TAPE','STOP_AT_END','OBJECT','RACE','CALIBRATE','MANUAL']);
  assert.ok(!html.includes('<form')&&!html.includes('maintenance')&&!html.includes('Calibration'));
  console.log('PASS browser events: connect, keys, combinations, release, modes, stop, blur, repeat, fields, visibility, sequence');
})().catch(error=>{console.error(error);process.exitCode=1;});
