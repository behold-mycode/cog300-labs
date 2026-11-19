"""Optional real-browser check: python -m pip install playwright; use installed Chrome."""
from pathlib import Path
import json
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from playwright.sync_api import sync_playwright, expect
html = (Path(__file__).parents[1]/'Robot/WirelessPage.h').read_text().split('R"HTML(',1)[1].rsplit(')HTML"',1)[0]
compressed=bytes(int(x,16) for x in re.findall(r'0x([0-9a-f]{2})',(Path(__file__).parents[1]/'Robot/WirelessPageGzip.h').read_text()))
class PageServer(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-Type','text/html; charset=utf-8')
        self.send_header('Content-Encoding','gzip')
        self.send_header('Content-Length',str(len(compressed)))
        self.end_headers();self.wfile.write(compressed)
    def log_message(self,*args):pass
server=ThreadingHTTPServer(('127.0.0.1',0),PageServer)
threading.Thread(target=server.serve_forever,daemon=True).start()
base=f'http://127.0.0.1:{server.server_port}'
with sync_playwright() as p:
    browser=p.chromium.launch(channel='chrome',headless=True)
    page=browser.new_page(viewport={'width':390,'height':844})
    errors=[];commands=[];state={'token':0,'owner':'none','mode':'MANUAL','state':'STOPPED','ir':2,'pwm':[0,0],'phase':0,'speed':[0,0],'radioMaxUs':1400,'tune':[250,250,250,125,200,18.82,.2353,.5647,61,66,0],'ms':100,'poseEpoch':1,'poseReady':True,'pose':[0,0,0]}
    page.on('pageerror',lambda error:errors.append(str(error)))
    def route(r):
        req=r.request
        if req.url.endswith('/'):
            r.continue_();return
        cmd=req.post_data
        if cmd: commands.append(cmd)
        ok=True
        if cmd=='CLAIM': state.update(token=123,owner='Wi-Fi')
        elif cmd=='STOP' or cmd=='RELEASE': state.update(token=0,owner='none',mode='MANUAL',state='STOPPED')
        elif cmd and req.headers.get('x-robot-token')!=str(state['token']): ok=False
        elif cmd=='MODE LINE':state.update(mode='LINE',state='LINE_FOLLOW')
        elif cmd and cmd.startswith('LINESPEED'):state['tune'][:4]=list(map(float,cmd.split()[1:]))
        state['ms']+=250
        if state['mode']=='LINE':state['pose']=[state['pose'][0]+20,state['pose'][1]+5,.2]
        response=dict(state,ok=ok)
        if not cmd:response['token']=0
        r.fulfill(status=200 if ok else 409,content_type='application/json',body=json.dumps(response))
    page.route(base+'/**',route)
    page.goto(base+'/');expect(page.locator('#linkStatus')).to_have_text('Live')
    expect(page.get_by_role('button',name='Start line following')).to_be_disabled()
    expect(page.locator('#irCentre')).to_be_visible()
    expect(page.locator('#irCentre')).to_have_text('TAPE')
    expect(page.locator('#irLeft')).to_have_text('floor')
    expect(page.locator('#irRight')).to_have_text('floor')
    page.get_by_role('button',name='Take Wi-Fi control').click()
    expect(page.get_by_role('button',name='Start line following')).to_be_enabled()
    page.get_by_role('button',name='Start line following').click()
    expect(page.locator('#status')).to_contain_text('LINE_FOLLOW')
    page.wait_for_function("document.querySelector('#status').textContent.includes('Wi-Fi')")
    state['ir']=4
    expect(page.locator('#irLeft')).to_have_text('TAPE')
    expect(page.locator('#irCentre')).to_have_text('floor')
    state['ir']=1
    expect(page.locator('#irRight')).to_have_text('TAPE')
    state['ir']=2
    expect(page.locator('#irCentre')).to_have_text('TAPE')
    page.evaluate('scrollTo(0,document.body.scrollHeight)')
    box=page.locator('#irCentre').bounding_box()
    assert box and 0 <= box['y'] < 844
    page.evaluate('scrollTo(0,0)')
    slider=page.get_by_role('slider',name='Driving speed')
    expect(slider).to_be_enabled()
    slider.fill('300');slider.dispatch_event('change')
    expect(page.locator('#message')).to_contain_text('Speeds applied')
    assert 'LINESPEED 300 250 300 150' in commands
    expect(page.locator('#speeds input[type=range]')).to_have_count(3)
    steering=page.get_by_role('slider',name='Steering strength')
    steering.fill('75');steering.dispatch_event('change')
    page.wait_for_function("snapshot.tune[2] === 375 && snapshot.tune[3] === 94")
    assert 'LINESPEED 300 250 375 94' in commands
    page.get_by_text('Run labels for later analysis',exact=True).click()
    page.get_by_label('Course name',exact=True).fill('Lab loop')
    page.get_by_label('Run result',exact=True).select_option('missed-turn')
    page.get_by_role('button',name='Mark missed turn',exact=True).click()
    expect(page.locator('#status')).to_contain_text('LINE_FOLLOW')
    page.wait_for_function("traceRecords.length >= 3")
    before=page.evaluate('traceRecords.length')
    page.wait_for_function(f"traceRecords.length > {before}")
    assert page.evaluate('traceRecords.some(p => p.x > 0)')
    page.get_by_role('button',name='Pause trail').click()
    before=page.evaluate('traceRecords.length')
    page.wait_for_timeout(700)
    assert page.evaluate('traceRecords.length')==before
    page.get_by_role('button',name='Resume trail').click()
    page.wait_for_function(f"traceRecords.length > {before}")
    with page.expect_download() as download:
        page.get_by_role('button',name='Download run').click()
    data=json.loads(Path(download.value.path()).read_text())
    assert data['units']['position']=='mm' and len(data['samples'])>0
    assert data['samples'][-1]['tune'][0]==300
    assert data['run']['course']=='Lab loop' and data['run']['outcome']=='missed-turn'
    assert data['events'][0]['kind']=='missed-turn'
    assert data['run']['id'] and data['format']=='cogs-robot-trace-v2'
    page.get_by_role('button',name='Clear trail').click()
    assert page.evaluate('traceRecords.length')<=1
    # An odometry reset begins a new frame instead of connecting unrelated poses.
    frame=page.evaluate('traceFrame')
    state.update(poseEpoch=2,pose=[0,0,0])
    page.wait_for_function(f"traceFrame > {frame}")
    page.get_by_role('button',name='STOP',exact=True).click()
    expect(page.locator('#message')).to_have_text('Stopped.')
    expect(page.get_by_role('button',name='Start line following')).to_be_disabled()
    expect(slider).to_be_disabled()
    # Page fits a narrow phone viewport and all script actions are error-free.
    assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
    assert not errors,errors
    page.screenshot(path='/tmp/cog300-wifi-page.png',full_page=True)
    # Lost request revokes the page's ownership instead of automatically claiming.
    page.get_by_role('button',name='Take Wi-Fi control').click()
    expect(page.get_by_role('button',name='Start line following')).to_be_enabled()
    page.route(base+'/command',lambda r:r.abort() if r.request.post_data=='HEARTBEAT' else route(r))
    expect(page.locator('#message')).to_contain_text('Connection lost',timeout=4000)
    expect(page.get_by_role('button',name='Start line following')).to_be_disabled()
    assert commands.count('CLAIM')==2
    expect(page.locator('#irCentre')).to_have_text('—')
    # An observer must not promise the active USB owner's lease will expire.
    state.update(owner='USB',token=0)
    page.get_by_role('button',name='STOP',exact=True).click()
    expect(page.locator('#message')).to_have_text('Stopped.')
    page.route(base+'/command',lambda r:r.abort())
    page.get_by_role('button',name='STOP',exact=True).click()
    expect(page.locator('#message')).to_contain_text('Use the controlling connection')
    browser.close()
server.shutdown();server.server_close()
print('PASS Wi-Fi browser: claim/start/stop, tuning, loss, mobile layout, no script errors')
