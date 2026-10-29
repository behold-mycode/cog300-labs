'use strict';
const $ = id => document.getElementById(id);
const token = new URLSearchParams(location.hash.slice(1)).get('token') || '';
// Keep the token in this tab's URL so reload works; it is never sent as a Referer.
let owner = null, seq = 0, renderedSeq = -1, ready = false, mode = 'MANUAL';
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
    button.disabled = !ready;
  });
  $('modeLabel').textContent = {MANUAL:'Manual driving',FOLLOW:'Following object',WALL:'Following right wall'}[mode];
}
function render(state) {
  if (!settingsLoaded && state.settings) {
    const fields = $('settingsForm').elements;
    for (const [name, value] of Object.entries(state.settings)) {
      if (typeof value === 'boolean') fields[name].checked = value;
      else fields[name].value = value;
    }
    settingsLoaded = true;
  }
  ready = Boolean(state.ready && owner);
  if (!ready) { pressed.clear(); mode = 'MANUAL'; paintKeys(); }
  $('badge').textContent = ready ? 'Connected' : state.connected ? 'Connecting' : 'Disconnected';
  $('status').textContent = state.status;
  $('connect').textContent = owner ? 'Disconnect' : 'Connect';
  $('connect').disabled = Boolean(state.connected && !owner);
  $('settingsForm').querySelector('button').disabled = !ready;
  const t = state.telemetry || {};
  const distance = value => Number.isFinite(value) && value >= 0 ? value.toFixed(1)+' cm' : '—';
  $('front').textContent = state.connected ? distance(t.front) : '—';
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
      const data = await api('connect', {port:$('port').value});
      owner = data.owner; seq = 0; renderedSeq = -1; localStop();
      await sendFrame(); $('drive').focus();
    }
  } catch (error) { showError(error); }
  finally { $('connect').disabled = false; }
});
$('stop').addEventListener('click', stop);
document.querySelectorAll('[data-mode]').forEach(button => button.addEventListener('click', () => {
  if (!ready) return;
  pressed.clear(); paintKeys(); mode = button.dataset.mode; modeUI();
  void sendFrame(); $('drive').focus();
}));
window.addEventListener('keydown', event => {
  if (event.code === 'Space' || event.code === 'Escape') {
    event.preventDefault(); stop(); return;
  }
  if (!mapping[event.code] || event.target.closest('input,select,textarea') || event.metaKey || event.ctrlKey || event.altKey) return;
  event.preventDefault();
  if (!ready || event.repeat || !document.hasFocus()) return;
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
  const settings = Object.fromEntries(['power','minimum','followCm','wallCm'].map(k => [k, Number(form[k].value)]));
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
