'use strict';
const $ = id => document.getElementById(id);
const token = new URLSearchParams(location.hash.slice(1)).get('token') || '';
let arenaCapable=false;
let owner=null, seq=0, renderedSeq=-1, ready=false, mode='MANUAL', framePending=false;
const pressed=new Set();
let meterUntil=0;
const mapping={ArrowUp:'Up',ArrowDown:'Down',ArrowLeft:'Left',ArrowRight:'Right',KeyW:'Up',KeyS:'Down',KeyA:'Left',KeyD:'Right'};
async function api(path,body){
  const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),1200);
  try{
    const response=await fetch('/api/'+path,{method:body===undefined?'GET':'POST',headers:{'X-Robot-Token':token,'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),signal:abort.signal});
    const data=await response.json();if(!response.ok){const error=new Error(data.error||'Connection failed');error.status=response.status;throw error;}return data;
  }finally{clearTimeout(timer);}
}
function showError(e){$('error').textContent=e.message||String(e);$('error').hidden=false;}
function keyNames(){
  const keys=new Set([...pressed].map(code=>mapping[code] || code));
  if(meterUntil && performance.now()<meterUntil)keys.add('Up');
  return [...keys];
}
function paintKeys(){const keys=keyNames();for(const key of ['Up','Down','Left','Right'])$('key'+key).classList.toggle('active',keys.includes(key));}
function localStop(){meterUntil=0;pressed.clear();mode='MANUAL';paintKeys();}
function render(state){
  ready=Boolean(state.ready&&owner);
  if(!ready && mode==='MANUAL')localStop();
  for(const id of ['keyUp','keyDown','keyLeft','keyRight','meterRun'])$(id).disabled=!ready;
  $('badge').textContent=ready?'Connected':state.connected?'Connecting':'Disconnected';
  $('status').textContent=state.status;
  $('connect').textContent=owner?'Disconnect':'Connect';
  $('connect').disabled=Boolean(state.connected&&!owner);
  arenaCapable=state.radar?.capable===true;
  $('start').disabled=!ready||($('mode').value==='OBJECT'&&!arenaCapable);
  paintKeys();
  const t=state.telemetry||{};
  if(state.mode==='MANUAL'&&t.mode==='MANUAL'&&!['STOPPED','DRIVING','TIMED_TEST'].includes(t.state))localStop();
  $('robotState').textContent=state.connected?(t.state||'Stopped').replaceAll('_',' ').toLowerCase():'Not connected';
  const line=t.lineControl;
  $('wheelStatus').textContent=state.connected&&line
    ? `Wheel PWM L ${t.left??0} · R ${t.motorRight??0} / 255. Approx. speed L ${Math.round(line.speedL)} · R ${Math.round(line.speedR)} mm/s.${line.saturated?' Power trim at limit.':''}`
    : 'Wheel feedback appears after connection.';
  const irLive=state.connected&&Array.isArray(t.ir);
  const tapeLevel=state.settings?.lineTapeLevel;
  ['irLeft','irCentre','irRight'].forEach((id,index)=>{
    const value=irLive?t.ir[index]:undefined;
    $(id).textContent=value===0||value===1
      ? `${value} · ${tapeLevel===0||tapeLevel===1?(value===tapeLevel?'TAPE':'FLOOR'):'raw'}` : '—';
  });
  $('irHelp').textContent=irLive
    ? `Live readings · ${tapeLevel===0||tapeLevel===1?`${tapeLevel} = tape, ${1-tapeLevel} = floor.`:'Tape polarity not set.'} Keep the robot stopped while adjusting sensors.`
    : 'No live readings. Connect the robot to calibrate.';
  for(const id of ['benchScan','armMaze','setWidth','setRadius','setServo','setStep']) $(id).disabled=!ready||!arenaCapable;
  $('armMaze').textContent='Start wall-maze course';
  const profile=state.radar?.settings||{};
  $('radarSettingsStatus').textContent=state.connected
    ? `${profile.minMm==null?'Target width unconfirmed':`Confirmed target width ${profile.minMm}–${profile.maxMm} mm`} · ${profile.radius==null?'Entry radius unconfirmed':`Confirmed entry radius ${profile.radius} mm`}.` : '';
  if(state.connected&&arenaCapable) $('radarSettingsStatus').textContent+=` Bench step ${profile.step||3}°.`;
  $('servoStatus').textContent=profile.rightUs==null?'':`Confirmed right / forward / left: ${profile.rightUs} / ${profile.centerUs} / ${profile.leftUs} µs.`;
  $('scannerSupport').hidden=!state.ready||arenaCapable;
  drawRadar(state.radar || {});
  $('demo').textContent=state.demo?'DEMO · NO HARDWARE':'';
}
function drawRadar(radar){
  const samples=radar.samples||[], lock=radar.lock;
  const echoes=samples.filter(sample=>sample[1]>0);
  let summary=radar.moving?'Robot moving · scan cleared.':'Waiting for scan data.';
  if(samples.length){
    summary=`Sweep ${radar.sweep} · ${echoes.length} echoes / ${samples.length} bearings · ${radar.age.toFixed(1)} s ago.`;
    if(lock)summary+=` Target ${Math.abs(lock[0])}° ${lock[0]<0?'right':'left'}, range ${lock[1]} mm, apparent width ${lock[2]} mm.`;
  }else if(radar.age!=null)summary=`Scan expired · last sample ${radar.age.toFixed(1)} s ago.`;
  $('radarSummary').textContent=summary;
  $('benchWidths').hidden=!Array.isArray(radar.bench);
  if(Array.isArray(radar.bench)){
    // Keep the preceding stationary sweep readable as the next one arrives.
    const scans=[{sweep:radar.sweep,contours:radar.bench},radar.previousBench].filter(Boolean);
    const items=[];
    for(const scan of scans){
      for(const contour of scan.contours){
        const item=document.createElement('li');
        const bearing=contour.bearing===0?'ahead':`${Math.abs(contour.bearing)}° ${contour.bearing>0?'left':'right'}`;
        item.textContent=`Sweep ${scan.sweep} · ${bearing} · range ${contour.range} mm · ${contour.status==='bounded'?'apparent width':'visible span'} ${contour.width} mm (${contour.status})`;
        items.push(item);
      }
    }
    if(items.length===0){
      const item=document.createElement('li');item.textContent='Waiting for adjacent returns.';items.push(item);
    }
    $('contourList').replaceChildren(...items);
  }
  const ctx=$('radarPlot').getContext?.('2d');
  if(!ctx)return;
  const outer=Math.max(1000,Math.ceil(Math.max(0,...echoes.map(s=>s[1]),lock?.[1]||0)/500)*500);
  const originX=320,originY=296,radius=268;
  const project=(a,r)=>[originX-Math.sin(a*Math.PI/180)*r/outer*radius,originY-Math.cos(a*Math.PI/180)*r/outer*radius];
  ctx.clearRect(0,0,640,332);ctx.lineWidth=1;ctx.strokeStyle='#486360';ctx.fillStyle='#b6cebf';ctx.font='13px sans-serif';
  for(const fraction of [.25,.5,.75,1]){
    ctx.beginPath();ctx.arc(originX,originY,radius*fraction,Math.PI,2*Math.PI);ctx.stroke();
    ctx.fillText(`${Math.round(outer*fraction)} mm`,originX+4,originY-radius*fraction+15);
  }
  for(const angle of [-90,-45,0,45,90]){
    ctx.beginPath();ctx.moveTo(originX,originY);ctx.lineTo(...project(angle,outer));ctx.stroke();
  }
  ctx.fillText('LEFT',38,319);ctx.fillText('ROBOT',298,319);ctx.fillText('RIGHT',561,319);
  const lockPoint=lock&&echoes.filter(p=>p[2]>0&&Math.abs(p[1]-lock[1])<=Math.max(40,.1*lock[1]))
    .sort((a,b)=>Math.abs(a[0]-lock[0])-Math.abs(b[0]-lock[0]))[0];
  const lockSegment=lockPoint&&Math.abs(lockPoint[0]-lock[0])<=5?lockPoint[2]:null;
  let previous=null;
  for(const [angle,distance,segment] of samples){
    if(distance<1){previous=null;continue;}
    const xy=project(angle,distance);
    ctx.fillStyle=segment===lockSegment?'#ffc574':'#87dfb5';ctx.strokeStyle=ctx.fillStyle;ctx.lineWidth=2;
    if(previous&&segment>0&&previous.segment===segment&&(Array.isArray(radar.bench)?Math.abs(previous.angle-angle)===(radar.settings?.step||3):[2,5].includes(Math.abs(previous.angle-angle)))){
      ctx.beginPath();ctx.moveTo(...previous.xy);ctx.lineTo(...xy);ctx.stroke();
    }
    ctx.beginPath();ctx.arc(...xy,3,0,2*Math.PI);ctx.fill();
    previous={xy,segment,angle};
  }
  if(lock){
    ctx.strokeStyle='#ffc574';ctx.lineWidth=3;ctx.beginPath();
    ctx.arc(...project(lock[0],lock[1]),8,0,2*Math.PI);ctx.stroke();
  }
}
async function sendFrame(){
  if(!owner)return;
  const requestOwner=owner,requestSeq=++seq;
  try{
    const state=await api('frame',{owner:requestOwner,seq:requestSeq,keys:keyNames(),mode,manualPower:Number($('manualPower').value)});
    if(owner===requestOwner&&requestSeq>=renderedSeq){renderedSeq=requestSeq;render(state);}
  }catch(error){
    if(requestOwner!==owner||requestSeq<seq)return;
    const ended=error.status>=400&&error.status<500;
    if(ended)owner=null;
    ready=false;if(ended||mode==='MANUAL')localStop();
    render({connected:!ended,ready:false,status:ended?error.message:'Waiting for controller — retrying; motion disabled'});
  }
}
function stop(){localStop();void sendFrame();}
async function refresh(){
  $('error').hidden=true;
  try{
    const ports=await api('ports');$('port').replaceChildren();
    for(const port of ports){const option=document.createElement('option');option.value=port.device;option.textContent=port.device+' · '+port.description;$('port').append(option);}
    if(!ports.length)$('status').textContent='No serial port found. Connect the robot and refresh.';
  }catch(error){showError(error);}
}
$('refresh').addEventListener('click',refresh);
$('connect').addEventListener('click',async()=>{
  $('connect').disabled=true;$('error').hidden=true;
  try{
    if(owner){const closing=owner;localStop();owner=null;ready=false;render(await api('disconnect',{owner:closing,seq:++seq}));}
    else{const data=await api('connect',{port:$('port').value});owner=data.owner;seq=0;renderedSeq=-1;localStop();await sendFrame();$('drive').focus();}
  }catch(error){showError(error);}finally{$('connect').disabled=false;}
});
$('mode').addEventListener('change',stop);
$('start').addEventListener('click',()=>{
  if(!ready||($('mode').value==='OBJECT'&&!arenaCapable))return;
  localStop();mode=$('mode').value;void sendFrame();$('drive').focus();
});
function startRadar(run){
  if(!ready||!arenaCapable)return;
  localStop();mode=run;void sendFrame();
}
$('radarStop').addEventListener('click',stop);
$('benchScan').addEventListener('click',()=>startRadar('RADAR_SCAN'));
$('armMaze').addEventListener('click',()=>startRadar('RADAR_ARM'));
async function setRadar(action,values){
  if(!ready)return;
  localStop();$('error').hidden=true;
  const current=owner,requestSeq=++seq;
  try{
    const state=await api('radar',{owner:current,seq:requestSeq,action,values});
    if(owner===current&&requestSeq>=renderedSeq){renderedSeq=requestSeq;render(state);}
  }catch(error){showError(error);void sendFrame();}
}
$('setWidth').addEventListener('click',()=>setRadar('radar_profile',{minMm:$('minWidth').valueAsNumber,maxMm:$('maxWidth').valueAsNumber}));
$('setStep').addEventListener('click',()=>setRadar('radar_step',{step:Number($('benchStep').value)}));
$('setServo').addEventListener('click',()=>setRadar('radar_servo',{rightUs:$('servoRight').valueAsNumber,centerUs:$('servoCenter').valueAsNumber,leftUs:$('servoLeft').valueAsNumber}));
$('setRadius').addEventListener('click',()=>setRadar('radar_clearance',{radius:$('clearRadius').valueAsNumber}));
$('manualPower').addEventListener('change',()=>{stop();$('drive').focus();});
function selectManual(){mode='MANUAL';$('mode').value=mode;}
for(const direction of ['Up','Down','Left','Right']){
  const button=$('key'+direction);
  button.addEventListener('pointerdown',event=>{
    if(!ready || event.button!==0 || !event.isPrimary)return;
    button.setPointerCapture(event.pointerId);
    selectManual();meterUntil=0;pressed.add(direction);paintKeys();void sendFrame();
  });
  function release(){
    if(pressed.delete(direction)){paintKeys();void sendFrame();}
  }
  for(const event of ['pointerup','pointercancel','lostpointercapture'])button.addEventListener(event,release);
}
$('meterRun').addEventListener('click',()=>{
  if(!ready)return;
  localStop();selectManual();meterUntil=performance.now()+3000;paintKeys();void sendFrame();
});
$('stop').addEventListener('click',stop);
window.addEventListener('keydown',event=>{
  if(event.code==='Space'||event.code==='Escape'){event.preventDefault();stop();return;}
  if(!mapping[event.code]||event.target.closest('input,select,textarea')||event.metaKey||event.ctrlKey||event.altKey)return;
  event.preventDefault();if(!ready||event.repeat||!document.hasFocus())return;
  meterUntil=0;pressed.add(event.code);mode='MANUAL';$('mode').value='MANUAL';paintKeys();void sendFrame();
});
window.addEventListener('keyup',event=>{if(mapping[event.code]&&pressed.delete(event.code)){paintKeys();void sendFrame();}});
function releaseManual(){if(mode==='MANUAL')stop();}
window.addEventListener('blur',releaseManual);
window.addEventListener('pagehide',releaseManual);
document.addEventListener('visibilitychange',()=>{if(document.hidden)releaseManual();});
document.addEventListener('focusin',event=>{if(event.target.closest('input,select,textarea'))stop();});
setInterval(async()=>{
  if(framePending)return;framePending=true;
  try{if(owner)await sendFrame();else render(await api('state'));}catch(error){showError(error);}finally{framePending=false;}
},100);
void refresh();
