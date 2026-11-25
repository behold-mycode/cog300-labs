#pragma once
static const char wirelessPage[]=R"HTML(<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>COGS car · live control</title>
<style>
:root{color-scheme:light;--ink:#152a3a;--blue:#145ca0;--line:#b7c6d0;--paper:#fff;--muted:#536574}
*{box-sizing:border-box}body{margin:0;background:#edf2f5;color:var(--ink);font:16px system-ui,sans-serif}main{max-width:920px;margin:auto;padding:20px}h1{font-size:26px;margin:0}h2{font-size:19px;margin:0 0 12px}p{line-height:1.5;margin:8px 0 16px}header{display:flex;justify-content:space-between;align-items:center;gap:12px;margin:8px 0 20px}.tag{font-size:13px;color:var(--muted)}section{background:var(--paper);padding:20px;margin:16px 0;border:1px solid var(--line);border-radius:10px}button,input{font:inherit}button{padding:10px 14px;border-radius:6px;border:1px solid var(--blue);background:var(--blue);color:white;cursor:pointer;min-height:44px}button.secondary{background:white;color:var(--blue)}button:disabled{opacity:.45;cursor:default}.actions{display:flex;flex-wrap:wrap;gap:10px}.drive{position:sticky;top:0;z-index:1;border-bottom:1px solid var(--line);background:#edf2f5;padding:10px 0}.drive button{flex:1}#stop{background:#b52228;border-color:#b52228;font-weight:750}#message{min-height:24px;margin:8px 0;color:var(--ink)}small,.hint{color:var(--muted);font-size:13px;line-height:1.5}#status{white-space:pre-wrap;font:13px/1.6 ui-monospace,monospace;margin-top:12px}#path{display:block;width:100%;height:300px;background:#f8fafb;border:1px solid var(--line);border-radius:6px}#pathMeta{margin:10px 0;font-size:13px;font-variant-numeric:tabular-nums}#linkStatus{font-weight:650;color:var(--blue)}.sliderRow{margin:16px 0}.sliderRow label{display:flex;justify-content:space-between;gap:12px;font-weight:550}.sliderRow output{font-variant-numeric:tabular-nums;color:var(--blue);white-space:nowrap}input[type=text],select,textarea{display:block;width:100%;font:inherit;padding:9px;border:1px solid var(--line);border-radius:5px;margin:6px 0 14px}input[type=range]{width:100%;height:38px;margin:0;padding:0;accent-color:var(--blue);cursor:pointer}fieldset{border:0;padding:0;margin:0;min-width:0}fieldset:disabled{opacity:.55}summary{cursor:pointer;font-weight:650;min-height:36px}button:focus-visible,input:focus-visible,summary:focus-visible{outline:3px solid #af6500;outline-offset:3px}.split{display:grid;grid-template-columns:1fr 1fr;gap:24px}@media(max-width:650px){main{padding:12px}section{padding:14px}.split{grid-template-columns:1fr;gap:0}.drive button{padding:9px 7px;font-size:14px}header{align-items:start}#path{height:270px}}
.irReadings{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin-top:8px}.irSensor{display:grid;text-align:center;gap:3px;padding:7px;border:1px solid var(--line);border-radius:6px;background:white;font-size:13px}.irSensor strong{font-size:17px}.irSensor[data-reading=tape]{background:var(--blue);border-color:var(--blue);color:white}.irSensor[data-reading=unknown]{background:#edf2f5;color:var(--muted)}
#radarPlot{display:block;width:100%;height:auto;background:#f8fafb;border:1px solid var(--line);border-radius:6px}#radarMeta{font-variant-numeric:tabular-nums}#radarProfile,#radarClearance{margin:12px 0}input[type=number]{display:block;width:100%;max-width:14rem;font:inherit;padding:8px;margin:4px 0 8px}#radarProfile .actions{align-items:center}td,th{text-align:left;padding:3px 12px}
</style>
<main><header><div><h1>COGS car</h1><span class="tag">Direct line follower · rollback</span></div><span id="linkStatus" role="status">Connecting…</span></header>
<div class="drive"><div class="actions"><button id="claim">Take Wi-Fi control</button><button id="start" disabled>Start line following</button><button id="stop">STOP</button></div><div class="irReadings" role="group" aria-label="Live IR sensor readings"><div class="irSensor" data-reading="unknown"><span>Left IR</span><strong id="irLeft">—</strong></div><div class="irSensor" data-reading="unknown"><span>Centre IR</span><strong id="irCentre">—</strong></div><div class="irSensor" data-reading="unknown"><span>Right IR</span><strong id="irRight">—</strong></div></div><p id="message" role="status" aria-live="polite"></p></div>
<p class="hint" id="controlHint">Take Wi-Fi control to enable tuning. Once started, the car follows the line until you press Stop, even if Wi-Fi drops.</p>
<section><h2>Object radar</h2>
<canvas id="radarPlot" width="600" height="260" role="img" aria-label="Vehicle-relative sonar returns, forward at top" aria-describedby="radarMeta"></canvas>
<p id="radarMeta">Waiting for scanner setup.</p>
<p id="radarWidths" hidden></p>
<details><summary>Measured bearings and ranges</summary><table><thead><tr><th scope="col">Bearing (° left)</th><th scope="col">Range (mm)</th></tr></thead><tbody id="radarRows"></tbody></table></details>
<fieldset disabled><legend>Scanner setup</legend>
<form id="radarStep"><label for="stepDegrees">Stationary scan spacing</label><select id="stepDegrees"><option value="1">1°</option><option value="2">2°</option><option value="3" selected>3°</option><option value="5">5°</option></select><button class="secondary">Apply spacing</button></form>
<form id="radarProfile"><p class="hint">Enter the apparent target widths measured during calibration.</p>
<div class="actions"><label for="radarMin">Minimum width (mm)<input id="radarMin" type="number" min="1" max="2000" required></label><label for="radarMax">Maximum width (mm)<input id="radarMax" type="number" min="1" max="2000" required></label><button>Set target profile</button></div></form>
<form id="radarClearance"><label for="radarRadius">Observed clear radius around entry axle (mm)</label><input id="radarRadius" type="number" min="0" max="2000" required><button class="secondary">Set entry clearance</button></form>
<details><summary>Servo and wheel calibration</summary><form id="radarServo"><p class="hint">Enter the measured pulse widths for this bracket.</p><div class="actions"><label for="servoRight">60° right (µs)<input id="servoRight" type="number" min="800" max="2200" required></label><label for="servoCentre">Forward (µs)<input id="servoCentre" type="number" min="800" max="2200" required></label><label for="servoLeft">60° left (µs)<input id="servoLeft" type="number" min="800" max="2200" required></label><button>Apply servo calibration</button></div></form><button id="radarCalibrate" class="secondary">Start moving wheel calibration</button></details>
<div class="actions"><button class="secondary" id="radarScan">Scan in place</button><button class="secondary" id="radarArm">Start wall-maze course</button></div></fieldset></section>
<section><h2>Live path</h2><canvas id="path" role="img" aria-label="Live estimated robot path and heading">Estimated path; numerical position is shown below.</canvas><div id="pathMeta">Waiting for encoder position…</div><div class="actions"><button class="secondary" id="record">Pause trail</button><button class="secondary" id="clear">Clear trail</button><button class="secondary" id="download" disabled>Download run</button><button class="secondary" id="mark" disabled>Mark missed turn</button></div><p class="hint">Updates during the run. Arrow = robot heading; the grid label shows the scale. This is an encoder estimate, not a map of the tape. Wheel slip causes drift. Recording stays in this page until downloaded.</p></section>
<section><details><summary>Run labels for later analysis</summary><label for="courseName">Course name</label><input id="courseName" type="text" maxlength="100" placeholder="e.g. Lab loop"><label for="runOutcome">Run result</label><select id="runOutcome"><option value="unlabelled">Not labelled</option><option value="completed">Completed successfully</option><option value="missed-turn">Missed a turn</option><option value="stopped">Stopped early</option></select><label for="runNotes">Run notes</label><textarea id="runNotes" maxlength="2000" rows="2" placeholder="Battery, floor, or where it failed"></textarea><p id="markers" class="hint" role="status">No missed turns marked.</p><p class="hint">Labels and button timestamps are observations, not proof of the correct driving action. Download the run before closing the page.</p></details></section><section><h2>Driving controls</h2><p class="hint">Release a slider to apply it over Wi-Fi, including while driving. No upload needed. Settings reset when robot power is switched off.</p><div id="speeds"></div><p class="hint" id="applied">Waiting for current settings…</p></section>
<section><details><summary>Motor calibration · advanced</summary><p class="hint">These controls stop the car when applied. Ordinary driving speed is the slider above.</p><div id="advanced"></div></details></section>
<section><details><summary>Live readings</summary><div id="status">Connecting…</div></details><button class="secondary" id="release" disabled>Release control</button></section></main>
<script>
const el=id=>document.getElementById(id);let token=0,busy=false,generation=0,snapshot=null,polling=false,queued=null,lastMs=0;
const groups=[['LINEMOTOR',[['Reference speed',30,1000,1],['Offset PWM',0,254,.01],['Proportional gain',0,5,.001],['Integral gain',0,10,.001],['Launch PWM',0,255,1]],4],['LINECFG',[['Reference PWM',1,255,1],['Minimum PWM',0,255,1]],9]];
const inputs=[],outputs=[],knobs=[];let basicsLoaded=false;
function showValue(i){outputs[i].value=inputs[i].value;inputs[i].setAttribute('aria-valuetext',inputs[i].value+(i===4?' mm/s':''))}
function resetTuning(){basicsLoaded=false;inputs.forEach(input=>delete input.dataset.loaded)}
function linkedSpeeds(){return [+knobs[0].value]}
const basicForm=document.createElement('form'),basicSet=document.createElement('fieldset');basicSet.disabled=true;
[['Driving speed',30,600,'mm/s','Straight-line speed. Steering, corner slowing and line recovery are automatic.']].forEach(([name,min,max,unit,hint],i)=>{const row=document.createElement('div'),label=document.createElement('label'),input=document.createElement('input'),output=document.createElement('output'),help=document.createElement('small');row.className='sliderRow';input.type='range';input.id='drive'+i;input.min=min;input.max=max;input.step=1;label.htmlFor=input.id;label.textContent=name;output.htmlFor=input.id;label.append(output);help.textContent=hint;const show=()=>{output.value=input.value+' '+unit;input.setAttribute('aria-valuetext',output.value)};input.addEventListener('input',show);input.addEventListener('change',()=>act('LINESPEED '+linkedSpeeds().join(' ')));knobs[i]=input;input.showValue=show;row.append(label,input,help);basicSet.append(row)});
const apply=document.createElement('button');apply.textContent='Apply driving settings';basicSet.append(apply);basicForm.append(basicSet);basicForm.onsubmit=e=>{e.preventDefault();act('LINESPEED '+linkedSpeeds().join(' '))};el('speeds').append(basicForm);
for(const [command,fields,offset] of groups){const form=document.createElement('form'),set=document.createElement('fieldset');set.disabled=true;const send=()=>act(command+' '+fields.map((_,i)=>inputs[offset+i].value).join(' '));fields.forEach(([name,min,max,step],j)=>{const i=offset+j,row=document.createElement('div'),label=document.createElement('label'),input=document.createElement('input'),output=document.createElement('output');row.className='sliderRow';input.type='range';input.id='tune'+i;input.min=min;input.max=max;input.step=step;label.htmlFor=input.id;label.textContent=name;output.htmlFor=input.id;label.append(output);inputs[i]=input;outputs[i]=output;input.addEventListener('input',()=>showValue(i));row.append(label,input);set.append(row)});const button=document.createElement('button');button.textContent=command==='LINEMOTOR'?'Apply motor calibration':'Apply PWM settings';set.append(button);form.append(set);form.addEventListener('submit',e=>{e.preventDefault();send()});el('advanced').append(form)}

function controls(){el('claim').disabled=busy||!!token;el('start').disabled=busy||!token;el('release').disabled=busy||!token;document.querySelectorAll('fieldset').forEach(f=>f.disabled=busy||!token);el('controlHint').textContent=token?'Control is yours. Speed sliders apply on release. Once started, the car follows the line until you press Stop.':'Take Wi-Fi control to enable tuning. Once started, the car follows the line until you press Stop, even if Wi-Fi drops.'}
let runId=Date.now().toString(36)+Math.random().toString(36).slice(2,8),runEvents=[];
let traceRecords=[],recording=true,traceFrame=0,traceSegment=0,origin=null,lastPose=null,lastRobotMs=null,lastEpoch=null,lastArrival=0,disconnected=true;
const MAX_SAMPLES=20000,canvas=el('path'),ctx=canvas.getContext('2d');
function clearTrace(){traceRecords=[];runEvents=[];runId=Date.now().toString(36)+Math.random().toString(36).slice(2,8);el("runOutcome").value="unlabelled";el("runNotes").value="";el("markers").textContent="No missed turns marked.";traceSegment++;origin=lastPose?[lastPose.rawX,lastPose.rawY]:null;if(lastPose){lastPose.x=0;lastPose.y=0}el('download').disabled=true;drawPath();pathText()}
function pathText(){el('pathMeta').textContent=lastPose?`x ${lastPose.x.toFixed(0)} mm · y ${lastPose.y.toFixed(0)} mm · heading ${(lastPose.heading*180/Math.PI).toFixed(0)}° · ${traceRecords.length} samples · ${recording?'recording':'trail paused'}`:'Waiting for encoder position…'}
function trace(s){const now=Date.now();lastArrival=now;el('linkStatus').textContent='Live';
 if(!s.poseReady||!Array.isArray(s.pose)||!s.pose.every(Number.isFinite)){el('pathMeta').textContent='Encoder position unavailable';return}
 if(lastRobotMs!==null&&(s.ms<lastRobotMs||s.poseEpoch!==lastEpoch)){traceFrame++;traceSegment++;origin=null;resetTuning()}
 if(disconnected){traceSegment++;disconnected=false}
 lastRobotMs=s.ms;lastEpoch=s.poseEpoch;if(!origin)origin=s.pose.slice(0,2);
 lastPose={ms:s.ms,receivedAt:now,frame:traceFrame,segment:traceSegment,poseEpoch:s.poseEpoch,x:s.pose[0]-origin[0],y:s.pose[1]-origin[1],heading:s.pose[2],rawX:s.pose[0],rawY:s.pose[1],ir:s.ir,pwm:s.pwm,speed:s.speed,state:s.state,bootCause:s.bootCause,resetFlags:s.resetFlags,radioRestarts:s.radioRestarts,tune:s.tune};
 if(recording){if(traceRecords.length>=MAX_SAMPLES){recording=false;el('record').textContent='Resume trail';el('message').textContent='Recording is full. Download the run, then clear the trail.'}else traceRecords.push(lastPose)}
 el('download').disabled=!traceRecords.length;el('mark').disabled=false;pathText();drawPath()}
function drawPath(){const w=canvas.clientWidth,h=canvas.clientHeight,dpr=window.devicePixelRatio||1;if(!w||!h)return;canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
 const points=traceRecords.filter(p=>p.frame===traceFrame);const bounds=lastPose?[...points,lastPose]:points;let xmin=-200,xmax=200,ymin=-150,ymax=150;for(const p of bounds){xmin=Math.min(xmin,p.x);xmax=Math.max(xmax,p.x);ymin=Math.min(ymin,p.y);ymax=Math.max(ymax,p.y)}const scale=Math.min((w-40)/(xmax-xmin),(h-40)/(ymax-ymin)),cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;const sx=x=>w/2+(x-cx)*scale,sy=y=>h/2-(y-cy)*scale;
 ctx.strokeStyle='#dce5eb';ctx.lineWidth=1;ctx.beginPath();const grid=Math.max(100,Math.ceil(12/scale/100)*100);for(let x=Math.ceil(xmin/grid)*grid;x<=xmax;x+=grid){ctx.moveTo(sx(x),20);ctx.lineTo(sx(x),h-20)}for(let y=Math.ceil(ymin/grid)*grid;y<=ymax;y+=grid){ctx.moveTo(20,sy(y));ctx.lineTo(w-20,sy(y))}ctx.stroke();
 ctx.strokeStyle='#145ca0';ctx.lineWidth=2.5;ctx.beginPath();let segment=-1;for(const p of points){if(p.segment!==segment){ctx.moveTo(sx(p.x),sy(p.y));segment=p.segment}else ctx.lineTo(sx(p.x),sy(p.y))}ctx.stroke();ctx.fillStyle='#536574';ctx.font='12px system-ui';ctx.fillText(`Grid ${grid/10} cm`,12,h-9);
 if(lastPose){ctx.save();ctx.translate(sx(lastPose.x),sy(lastPose.y));ctx.rotate(-lastPose.heading);ctx.fillStyle=disconnected?'#687780':'#b52228';ctx.beginPath();ctx.moveTo(12,0);ctx.lineTo(-8,-7);ctx.lineTo(-4,0);ctx.lineTo(-8,7);ctx.closePath();ctx.fill();ctx.restore()}}
el('mark').onclick=()=>{if(!lastPose)return;runEvents.push({kind:'missed-turn',ms:lastPose.ms,frame:lastPose.frame,receivedAt:Date.now()});el('markers').textContent=runEvents.length+' missed turn(s) marked.'};
el('clear').onclick=clearTrace;el('record').onclick=()=>{recording=!recording;traceSegment++;el('record').textContent=recording?'Pause trail':'Resume trail';pathText()};
el('download').onclick=()=>{const blob=new Blob([JSON.stringify({format:'cogs-robot-trace-v2',run:{id:runId,course:el('courseName').value,outcome:el('runOutcome').value,notes:el('runNotes').value},events:runEvents,units:{position:'mm',heading:'radians',robotTime:'ms'},note:'Encoder estimate; frames have separate origins; segments break on connection loss or paused recording.',samples:traceRecords},null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='COGS-Robot-Run-'+new Date().toISOString().replace(/[:.]/g,'-')+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};
new ResizeObserver(drawPath).observe(canvas);
function renderIR(mask){['irLeft','irCentre','irRight'].forEach((id,i)=>{const reading=mask==null?'unknown':mask&(4>>i)?'tape':'floor',value=reading==='unknown'?'—':reading==='tape'?'TAPE':'floor',node=el(id);if(node.textContent!==value)node.textContent=value;node.parentElement.dataset.reading=reading})}
const radarPlot=el('radarPlot'),radarCtx=radarPlot.getContext('2d');
let radarSweep=null,radarPoints=new Map(),radarBatch=0,radarBlocked=null;
function renderRadar(s){
 const packet=s?.radar,key=packet?[s.poseEpoch,packet.sweep].join('/'):null;
 const restarted=s&&s.ms<lastMs;
 if(restarted)radarBlocked=null;
 const moved=s&&snapshot&&s.poseEpoch===snapshot.poseEpoch&&!restarted&&s.pose?.some((v,i)=>v!==snapshot.pose?.[i]);
 if(key&&(s.pwm?.some(v=>v!==0)||moved))radarBlocked=key;
 const active=s?.mode==='OBJECT'||s?.mode==='RADAR_SCAN';
 const fresh=packet&&((s.ms-packet.ms)>>>0)<=2000;
 const r=active&&fresh&&key!==radarBlocked&&packet.points?.length?packet:null;
 if(!r||key!==radarSweep||restarted){radarPoints.clear();radarSweep=key;radarBatch=0}
 if(r){
  const newest=r.points||[];
  if(newest.length&&!newest.some(p=>radarPoints.has(p[0])))radarBatch++;
  for(const [angle,range,segment] of newest){
   const batch=radarPoints.get(angle)?.batch??radarBatch;
   radarPoints.set(angle,{angle,range,segment,batch});
  }
 }
 const points=[...radarPoints.values()],lock=r?.lock||[],age=r?((s.ms-r.ms)>>>0):0;
 const extent=Math.max(500,...points.map(p=>p.range),lock[1]||0);
 const w=radarPlot.width,h=radarPlot.height,x=w/2,y=h-25,k=Math.min((x-12)/extent,(y-15)/extent);
 const xy=(a,d)=>[x-Math.sin(a*Math.PI/180)*d*k,y-Math.cos(a*Math.PI/180)*d*k];
 const c=radarCtx;c.clearRect(0,0,w,h);c.strokeStyle='#bac9d2';c.lineWidth=1;
 for(const fraction of [.5,1]){c.beginPath();c.arc(x,y,extent*k*fraction,Math.PI,2*Math.PI);c.stroke()}
 c.font='15px system-ui';c.fillStyle='#536574';c.fillText('Forward',x+5,17);c.fillText('Left',10,y+15);c.fillText('Right',w-48,y+15);
 c.fillText(Math.round(extent)+' mm',x+6,y-extent*k+30);
 c.strokeStyle=c.fillStyle='#145ca0';c.lineWidth=3;let previous;
 for(const p of points){
  if(p.range<=0||p.segment<=0){previous=null;continue}
  const [px,py]=xy(p.angle,p.range);
  if(previous&&previous.segment===p.segment&&previous.batch===p.batch){c.beginPath();c.moveTo(...xy(previous.angle,previous.range));c.lineTo(px,py);c.stroke()}
  c.beginPath();c.arc(px,py,3,0,2*Math.PI);c.fill();previous=p;
 }
 if(lock.length===3){const [px,py]=xy(lock[0],lock[1]);c.strokeStyle='#9d341c';c.lineWidth=3;c.strokeRect(px-7,py-7,14,14)}
 const widths=[];
 if(r && s.mode==='RADAR_SCAN' && r.step>0){
  const sorted=points.slice().sort((a,b)=>a.angle-b.angle);
  for(let i=0;i<sorted.length;){
   const first=sorted[i++];
   if(first.range<=0 || first.segment<=0)continue;
   let end=first,sum=first.range,count=1;
   while(i<sorted.length && sorted[i].angle===end.angle+r.step && sorted[i].batch===first.batch &&
         sorted[i].segment===first.segment && sorted[i].range>0){
    end=sorted[i++];sum+=end.range;count++;
   }
   if(count<2)continue;
   const before=radarPoints.get(first.angle-r.step),after=radarPoints.get(end.angle+r.step);
   if(before?.batch!==first.batch || after?.batch!==first.batch)continue;
   const distance=sum/count,width=2*distance*Math.sin(count*r.step*Math.PI/360);
   widths.push(Math.round(width)+' mm wide at '+Math.round(distance)+' mm');
  }
 }
 const widthsNode=el('radarWidths');widthsNode.hidden=!(r&&s.mode==='RADAR_SCAN');
 widthsNode.textContent=widths.length?'Measured contours · '+widths.join(' / '):'Waiting for both edges of a contour.';
 el('radarMeta').textContent=!s?'Scanner updates interrupted.':!packet?'Waiting for scanner setup.':!r?'No current stationary sweep · '+s.state:
  'Sweep '+r.sweep+' · '+points.filter(p=>p.range>0).length+' returns · '+(age/1000).toFixed(1)+' s since sample · '+s.state+
  (lock.length===3?' · Target □ '+lock[0]+'°, '+lock[1]+' mm away, '+lock[2]+' mm wide':' · No target lock');
 const rows=points.map(p=>{const tr=document.createElement('tr');for(const v of [p.angle,p.range>0?p.range:'No return']){const td=document.createElement('td');td.textContent=v;tr.append(td)}return tr});el('radarRows').replaceChildren(...rows);
}
el('radarStep').onsubmit=e=>{e.preventDefault();act('RADAR STEP '+el('stepDegrees').value)};
el('radarMin').oninput=()=>el('radarMax').min=el('radarMin').value||1;
el('radarProfile').onsubmit=e=>{e.preventDefault();act('RADAR PROFILE '+el('radarMin').value+' '+el('radarMax').value)};
el('radarClearance').onsubmit=e=>{e.preventDefault();act('RADAR CLEARANCE '+el('radarRadius').value)};
el('radarServo').onsubmit=e=>{e.preventDefault();act('RADAR SERVO '+['servoRight','servoCentre','servoLeft'].map(id=>el(id).value).join(' '))};
el('radarCalibrate').onclick=()=>act('MODE CALIBRATE');
el('radarScan').onclick=()=>act('RADAR SCAN');el('radarArm').onclick=()=>act('RADAR ARM');
function render(s){renderRadar(s);if(lastMs&&s.ms<lastMs){token=0;el('message').textContent=`The car restarted. Reset flag: ${s.bootCause}.`}lastMs=s.ms;snapshot=s;trace(s);renderIR(s.ir);el('status').textContent=`Board uptime: ${Math.floor(s.ms/1000)} s\nLast boot: ${s.bootCause} · flags ${s.resetFlags}\nRadio restart attempts: ${s.radioRestarts}\nControl: ${s.owner}\n${s.mode} · ${s.state} · phase ${s.phase}\nTape L / C / R: ${[4,2,1].map(b=>s.ir&b?'TAPE':'floor').join(' / ')}\nWheel speed: ${s.speed.join(' / ')} mm/s\nMotor PWM: ${s.pwm.join(' / ')}\nLongest steering interval: ${(s.maxLineGapUs/1000).toFixed(2)} ms\nLongest radio service: ${(s.radioMaxUs/1000).toFixed(2)} ms`;
 if(!basicsLoaded){knobs[0].value=s.tune[0];knobs.forEach(input=>input.showValue());basicsLoaded=true}inputs.forEach((input,i)=>{if(!input.dataset.loaded){input.value=s.tune[i];input.dataset.loaded='1';showValue(i)}});el('applied').textContent='On robot: driving '+s.tune[0]+' mm/s · scan '+Math.min(150,s.tune[1])+' mm/s · confirmation '+Math.max(30,Math.min(125,s.tune[3]))+' mm/s (automatic).'}
async function request(command,ownedToken=token,keepalive=false){const response=await fetch(command?'/command':'/status',{method:command?'POST':'GET',headers:command?{'X-Robot-Control':'1','X-Robot-Token':String(ownedToken),'Content-Type':'text/plain'}:{},body:command||undefined,cache:'no-store',signal:AbortSignal.timeout(1100),keepalive});const s=await response.json();if(!response.ok||!s.ok){const e=new Error(s.owner==='USB'?'USB owns control. Disconnect its controller first.':`Command rejected: ${s.state}. Take control again if the session expired.`);e.rejected=true;e.status=s;throw e}return s}
function markLost(){disconnected=true;renderRadar(null);renderIR(null);el('linkStatus').textContent='Connection lost';drawPath()}
async function act(command){if(polling){queued=command;return}if(busy)return;busy=true;controls();const epoch=generation;try{const s=await request(command);if(epoch!==generation)return;token=s.token;if(command==='CLAIM')resetTuning();render(s);el('message').textContent=command==='CLAIM'?'Wi-Fi control ready.':command==='MODE CALIBRATE'?s.state:command.startsWith('RADAR ')?s.state:command==='MODE LINE'?'Line following started.':command.startsWith('LINESPEED')?'Speeds applied.':command==='RELEASE'?'Control released.':'Settings applied; car stopped.'}catch(e){if(epoch===generation){if(e.rejected){token=e.status.token||0;render(e.status);el('message').textContent=e.message}else{markLost();el('message').textContent='No reply from the car; retrying. Use Stop to end the run.'}}}finally{if(epoch===generation){busy=false;controls()}}}
async function stop(){const owned=!!token;generation++;queued=null;token=0;busy=false;controls();el('message').textContent='Sending Stop…';const epoch=generation;try{const s=await request('STOP',0,true);if(epoch===generation){render(s);el('message').textContent='Stopped.'}}catch(e){if(epoch===generation){markLost();el('message').textContent='Stop not confirmed. Press Stop again, or pick the car up.'}}}
el('claim').onclick=()=>act('CLAIM');el('start').onclick=()=>act('MODE LINE');el('release').onclick=()=>act('RELEASE');el('stop').onclick=stop;
async function poll(){if(!busy&&!document.hidden){polling=true;const epoch=generation;try{const s=await request(token?'HEARTBEAT':null);if(epoch===generation){if(token)token=s.token;render(s)}}catch(e){if(epoch===generation){markLost();if(e.rejected){token=0;el('message').textContent='Control ended. Take control again before starting.'}else el('message').textContent='Connection lost; retrying. A running car keeps driving until Stop.'}}finally{polling=false;if(epoch===generation){controls();if(queued){const command=queued;queued=null;act(command)}}}}setTimeout(poll,250)}
setInterval(()=>{if(lastArrival&&Date.now()-lastArrival>1500){disconnected=true;renderRadar(null);renderIR(null);el('linkStatus').textContent='Waiting for updates';drawPath()}},500);
document.addEventListener('visibilitychange',()=>{if(document.hidden)disconnected=true});controls();poll();
</script></html>)HTML";
