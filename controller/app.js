'use strict';
const $ = id => document.getElementById(id);
const token = new URLSearchParams(location.hash.slice(1)).get('token') || '';
// Keep the token in this tab's URL so reload works; it is never sent as a Referer.
let owner = null, seq = 0, renderedSeq = -1, ready = false, mode = 'MANUAL';
let navStatus = {}, mapDownloadPending = false, mapExporting = false;
let framePending = false, configurationPending = false, settingsLoaded = false, lastLog = '';
const pressed = new Set();
const mapping = {ArrowUp:'Up',ArrowDown:'Down',ArrowLeft:'Left',ArrowRight:'Right',
                 KeyW:'Up',KeyS:'Down',KeyA:'Left',KeyD:'Right'};

async function api(path, body) {
  const abort = new AbortController();
  const timer = setTimeout(() => abort.abort(), 1200);
  try {
    const response = await fetch('/api/' + path, {
      method: body === undefined ? 'GET' : 'POST',
      headers: {'X-Robot-Token':token, 'Content-Type':'application/json'},
      body: body === undefined ? undefined : JSON.stringify(body), signal:abort.signal
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || 'Connection failed');
    return data;
  } finally { clearTimeout(timer); }
}
function showError(error) { $('error').textContent = error.message || String(error); $('error').hidden = false; }
function clearError() { $('error').hidden = true; }
function keyNames() { return [...new Set([...pressed].map(code => mapping[code]))]; }
function paintKeys() {
  const keys = keyNames();
  for (const key of ['Up','Down','Left','Right']) $('key'+key).classList.toggle('active', keys.includes(key));
}
function modeUI() {
  document.querySelectorAll('[data-mode]').forEach(button => {
    const selected = button.dataset.mode === mode;
    button.classList.toggle('selected', selected);
    button.setAttribute('aria-pressed', String(selected));
    button.disabled = !ready || mapExporting || (button.dataset.mode === 'MAZE' && !(navStatus.encoders && navStatus.odometry && navStatus.geometry && navStatus.localized && navStatus.dynamics));
  });
  $('modeLabel').textContent = {MANUAL:'Manual driving',FOLLOW:'Following object',WALL:'Following right wall',LINE:'Following tape',MAZE:'Exploring maze',STOP_ON_TAPE:'Stop on tape',STOP_AT_END:'Stop at tape end'}[mode];
}
function render(state) {
  if (!settingsLoaded && state.settings) {
    const fields = $('settingsForm').elements;
    for (const [name, value] of Object.entries(state.settings)) {
      if (typeof value === 'boolean') fields[name].checked = value;
      else fields[name].value = value;
    }
    settingsLoaded = Boolean(state.ready);
  }
  ready = Boolean(state.ready && owner);
  if (!ready) { pressed.clear(); mode = 'MANUAL'; paintKeys(); }
  $('badge').textContent = ready ? 'Connected' : state.connected ? 'Connecting' : 'Disconnected';
  $('status').textContent = state.status;
  $('connect').textContent = owner ? 'Disconnect' : 'Connect';
  $('connect').disabled = Boolean(state.connected && !owner);
  $('settingsForm').querySelector('button').disabled = !ready;
  const t = state.telemetry || {};
  navStatus = {...(state.navigation || {}), dynamics:Boolean(t.dynamics)};
  mapExporting = Boolean(state.mapExporting);
  if (!state.connected) mapDownloadPending = false;
  if (state.mode === 'MANUAL' && t.mode === 'MANUAL' && !['STOPPED','DRIVING','TIMED_TEST'].includes(t.state)) mode = 'MANUAL';
  $('navReadiness').textContent = !state.connected ? 'Connect to read navigation readiness.' :
    `Encoders: ${navStatus.encoders ? 'enabled' : 'not enabled'} · Wheel scale: ${navStatus.odometry ? 'set' : 'required'} · Geometry: ${navStatus.geometry ? 'set' : 'required'} · Localization: ${navStatus.localized ? 'valid' : 'new map or relocalization required'}`;
  $('traceStatus').textContent = `${state.traceRecording?'Recording':'Recorded'}: ${state.traceRows || 0} / 20,000 observation samples`;
  $('beliefStatus').textContent = t.belief ? `Tape hypotheses L / centre / R / wide / missing: ${t.belief.map(v=>(100*v).toFixed(0)+'%').join(' / ')} · Braking model: ${t.dynamics?'set':'required'} · Nominal sensor probabilities: validate on robot` : 'Waiting for estimator telemetry';
  $('encoderCounts').textContent = t.encoderCounts ? `Counts — left: ${t.encoderCounts[0]} · right: ${t.encoderCounts[1]}` : 'Encoder counts: waiting';
  $('mapStatus').textContent = `Nodes: ${navStatus.nodes || 0} / 64 · Edges: ${navStatus.edges || 0} / 128 · Current node: ${navStatus.current ?? '—'} · Candidate revisit: ${navStatus.candidate ?? '—'}`;
  if (mapDownloadPending && state.mazeMap?.complete) {
    mapDownloadPending = false;
    downloadText(JSON.stringify(state.mazeMap,null,2)+'\n','robot-maze.json','application/json');
  }
  const distance = value => Number.isFinite(value) && value >= 0 ? value.toFixed(1)+' cm' : '—';
  $('front').textContent = state.connected ? distance(t.front) : '—';
  $('lineSensors').textContent = state.connected && t.ir ? t.ir.map(v => t.tapeLevel < 0 ? String(v) : v === t.tapeLevel ? 'TAPE' : 'ground').join(' / ') : '—';
  $('right').textContent = state.connected ? distance(t.right) : '—';
  $('robotState').textContent = state.connected ? (t.state || 'Waiting').replaceAll('_',' ').toLowerCase() : 'Not connected';
  if (state.demo) $('demo').textContent = 'DEMO · NO HARDWARE';
  lastLog = (state.log || []).join('\n');
  if ($('diagnostics').open) { $('log').textContent = lastLog; $('log').scrollTop = $('log').scrollHeight; }
  modeUI();
}
function localStop() { pressed.clear(); mode = 'MANUAL'; paintKeys(); modeUI(); }
async function sendFrame() {
  if (!owner || configurationPending) return;
  const requestOwner = owner, requestSeq = ++seq;
  try {
    const state = await api('frame', {owner:requestOwner, seq:requestSeq, keys:keyNames(), mode});
    if (owner === requestOwner && requestSeq >= renderedSeq) {
      renderedSeq = requestSeq; render(state);
    }
  } catch (error) {
    // An old frame rejected after a newer stop/settings request is harmless.
    if (requestOwner !== owner || requestSeq < seq) return;
    owner = null; ready = false; localStop();
    render({connected:false, ready:false, status:error.message});
  }
}
function stop() { localStop(); void sendFrame(); }
async function refresh() {
  clearError();
  try {
    const ports = await api('ports');
    $('port').replaceChildren();
    for (const port of ports) {
      const option = document.createElement('option'); option.value = port.device;
      option.textContent = port.device + ' · ' + port.description; $('port').append(option);
    }
    if (!ports.length) $('status').textContent = 'No serial ports found. Plug in the robot, then Refresh.';
  } catch (error) { showError(error); }
}
$('refresh').addEventListener('click', refresh);
$('connect').addEventListener('click', async () => {
  $('connect').disabled = true; clearError();
  try {
    if (owner) {
      const closing = owner; localStop(); owner = null; ready = false;
      const state = await api('disconnect', {owner:closing, seq:++seq}); render(state);
    } else {
      settingsLoaded=false;
      const data = await api('connect', {port:$('port').value});
      owner = data.owner; seq = 0; renderedSeq = -1; localStop();
      await sendFrame(); $('drive').focus();
    }
  } catch (error) { showError(error); }
  finally { $('connect').disabled = false; }
});
$('stop').addEventListener('click', stop);
document.querySelectorAll('[data-mode]').forEach(button => button.addEventListener('click', () => {
  if (!ready || mapExporting || button.disabled) return;
  pressed.clear(); paintKeys(); mode = button.dataset.mode; modeUI();
  void sendFrame(); $('drive').focus();
}));
window.addEventListener('keydown', event => {
  if (event.code === 'Space' || event.code === 'Escape') {
    event.preventDefault(); stop(); return;
  }
  if (!mapping[event.code] || event.target.closest('input,select,textarea') || event.metaKey || event.ctrlKey || event.altKey) return;
  event.preventDefault();
  if (!ready || mapExporting || event.repeat || !document.hasFocus()) return;
  pressed.add(event.code); mode = 'MANUAL'; paintKeys(); modeUI(); void sendFrame();
});
window.addEventListener('keyup', event => {
  if (!mapping[event.code]) return;
  if (pressed.delete(event.code)) { paintKeys(); void sendFrame(); }
});
window.addEventListener('blur', stop);
document.addEventListener('visibilitychange', () => { if (document.hidden) stop(); });
window.addEventListener('pagehide', stop);
$('settings').addEventListener('toggle', () => { if ($('settings').open) stop(); });
document.addEventListener('focusin', event => { if (event.target.closest('input,select,textarea')) stop(); });
$('settingsForm').addEventListener('submit', async event => {
  event.preventDefault(); if (!owner || !ready) return;
  clearError(); localStop();
  const form = event.target.elements;
  const settings = Object.fromEntries(['power','minimum','followCm','wallCm','lineTapeLevel'].map(k => [k, Number(form[k].value)]));
  settings.invertA = form.invertA.checked; settings.invertB = form.invertB.checked;
  const requestSeq = ++seq;
  configurationPending = true; ready = false; modeUI();
  try {
    const state = await api('settings', {owner, seq:requestSeq, settings});
    if (requestSeq >= renderedSeq) { renderedSeq = requestSeq; render(state); }
    $('drive').focus();
  } catch (error) { showError(error); }
  finally { configurationPending = false; }
});
function downloadText(text, filename, mime) {
  const url=URL.createObjectURL(new Blob([text],{type:mime}));
  const a=document.createElement('a');a.href=url;a.download=filename;a.click();
  setTimeout(()=>URL.revokeObjectURL(url),1000);
}
async function navigationAction(action, values={}) {
  if (!owner || !ready) { showError(new Error('Connect before configuring navigation.')); return; }
  localStop();configurationPending=true;clearError();
  const requestSeq=++seq;
  try {
    const state=await api('navigation',{owner,seq:requestSeq,action,values});
    if(requestSeq>=renderedSeq) { renderedSeq=requestSeq;render(state); }
  } catch(error) { mapDownloadPending=false;showError(error); }
  finally { configurationPending=false; }
}
for(const [id,action,fields] of [
  ['dynamicsForm','dynamics',['deceleration','reaction','margin']],
  ['modelForm','model',['hit','falsePositive','change']],
  ['encoderForm','encoders',['kind','invertL','invertR']],
  ['odometryForm','odometry',['left','right','track']],
  ['geometryForm','geometry',['offset','width','match','clearance']],
  ['confirmForm','map_confirm',['node']]]) {
  $(id).addEventListener('submit',event=>{
    event.preventDefault();
    const values=Object.fromEntries(fields.map(name=>[name,Number(event.target.elements[name].value)]));
    void navigationAction(action,values);
  });
}
$('exportMap').addEventListener('click',()=>{mapDownloadPending=true;void navigationAction('map_export');});
$('resetMap').addEventListener('click',()=>{
  if(window.confirm('Clear the current maze graph and set the current position as the new origin?')) void navigationAction('map_reset');
});
$('navigation').addEventListener('toggle',()=>{if($('navigation').open) stop();});
$('diagnostics').addEventListener('toggle', () => { $('log').textContent = lastLog; });
$('download').addEventListener('click', () => {
  stop();
  const url = URL.createObjectURL(new Blob([lastLog+'\n'], {type:'text/plain'}));
  const a = document.createElement('a'); a.href = url; a.download = 'robot-log.txt'; a.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
});
setInterval(async () => {
  if (framePending || configurationPending) return;
  framePending = true;
  try {
    if (owner) await sendFrame();
    else render(await api('state'));
  } catch (error) { showError(error); }
  finally { framePending = false; }
}, 100);
void refresh(); modeUI();

$('startTrace').addEventListener('click',()=>void navigationAction('trace',{enabled:1}));
$('stopTrace').addEventListener('click',()=>void navigationAction('trace',{enabled:0}));
$('downloadTrace').addEventListener('click',async()=>{
  stop();try { const data=await api('trace');downloadText(data.csv,'robot-observations.csv','text/csv'); }
  catch(error) { showError(error); }
});
