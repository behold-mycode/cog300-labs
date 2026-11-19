'use strict';
const $ = id => document.getElementById(id);
const token = new URLSearchParams(location.hash.slice(1)).get('token') || '';
let owner=null, seq=0, renderedSeq=-1, ready=false, mode='MANUAL', framePending=false;
const pressed=new Set();
const mapping={ArrowUp:'Up',ArrowDown:'Down',ArrowLeft:'Left',ArrowRight:'Right',KeyW:'Up',KeyS:'Down',KeyA:'Left',KeyD:'Right'};
async function api(path,body){
  const abort=new AbortController(),timer=setTimeout(()=>abort.abort(),1200);
  try{
    const response=await fetch('/api/'+path,{method:body===undefined?'GET':'POST',headers:{'X-Robot-Token':token,'Content-Type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),signal:abort.signal});
    const data=await response.json();if(!response.ok){const error=new Error(data.error||'Connection failed');error.status=response.status;throw error;}return data;
  }finally{clearTimeout(timer);}
}
function showError(e){$('error').textContent=e.message||String(e);$('error').hidden=false;}
function keyNames(){return [...new Set([...pressed].map(code=>mapping[code]))];}
function paintKeys(){const keys=keyNames();for(const key of ['Up','Down','Left','Right'])$('key'+key).classList.toggle('active',keys.includes(key));}
function localStop(){pressed.clear();mode='MANUAL';paintKeys();}
function render(state){
  ready=Boolean(state.ready&&owner);
  if(!ready)localStop();
  $('badge').textContent=ready?'Connected':state.connected?'Connecting':'Disconnected';
  $('status').textContent=state.status;
  $('connect').textContent=owner?'Disconnect':'Connect';
  $('connect').disabled=Boolean(state.connected&&!owner);
  $('start').disabled=!ready;
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
  $('demo').textContent=state.demo?'DEMO · NO HARDWARE':'';
}
async function sendFrame(){
  if(!owner)return;
  const requestOwner=owner,requestSeq=++seq;
  try{
    const state=await api('frame',{owner:requestOwner,seq:requestSeq,keys:keyNames(),mode});
    if(owner===requestOwner&&requestSeq>=renderedSeq){renderedSeq=requestSeq;render(state);}
  }catch(error){
    if(requestOwner!==owner||requestSeq<seq)return;
    const ended=error.status>=400&&error.status<500;
    if(ended)owner=null;
    ready=false;localStop();
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
  if(!ready)return;
  pressed.clear();paintKeys();mode=$('mode').value;void sendFrame();$('drive').focus();
});
$('stop').addEventListener('click',stop);
window.addEventListener('keydown',event=>{
  if(event.code==='Space'||event.code==='Escape'){event.preventDefault();stop();return;}
  if(!mapping[event.code]||event.target.closest('input,select,textarea')||event.metaKey||event.ctrlKey||event.altKey)return;
  event.preventDefault();if(!ready||event.repeat||!document.hasFocus())return;
  pressed.add(event.code);mode='MANUAL';$('mode').value='MANUAL';paintKeys();void sendFrame();
});
window.addEventListener('keyup',event=>{if(mapping[event.code]&&pressed.delete(event.code)){paintKeys();void sendFrame();}});
window.addEventListener('blur',stop);
window.addEventListener('pagehide',stop);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
document.addEventListener('focusin',event=>{if(event.target.closest('input,select,textarea'))stop();});
setInterval(async()=>{
  if(framePending)return;framePending=true;
  try{if(owner)await sendFrame();else render(await api('state'));}catch(error){showError(error);}finally{framePending=false;}
},100);
void refresh();
