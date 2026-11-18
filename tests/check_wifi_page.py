"""Optional real-browser check: python -m pip install playwright; use installed Chrome."""
from pathlib import Path
import json
from playwright.sync_api import sync_playwright, expect
html = (Path(__file__).parents[1]/'Robot/WirelessPage.h').read_text().split('R"HTML(',1)[1].rsplit(')HTML"',1)[0]
with sync_playwright() as p:
    browser=p.chromium.launch(channel='chrome',headless=True)
    page=browser.new_page(viewport={'width':390,'height':844})
    errors=[];commands=[];state={'token':0,'owner':'none','mode':'MANUAL','state':'STOPPED','ir':2,'pwm':[0,0],'phase':0,'speed':[0,0],'radioMaxUs':1400,'tune':[200,90,180,130,200,160,.12,.8,220,240,0]}
    page.on('pageerror',lambda error:errors.append(str(error)))
    def route(r):
        req=r.request
        if req.url.endswith('/'):
            r.fulfill(content_type='text/html',body=html);return
        cmd=req.post_data
        if cmd: commands.append(cmd)
        ok=True
        if cmd=='CLAIM': state.update(token=123,owner='Wi-Fi')
        elif cmd=='STOP' or cmd=='RELEASE': state.update(token=0,owner='none',mode='MANUAL',state='STOPPED')
        elif cmd and req.headers.get('x-robot-token')!=str(state['token']): ok=False
        elif cmd=='MODE LINE':state.update(mode='LINE',state='LINE_FOLLOW')
        elif cmd and cmd.startswith('LINESPEED'):state.update(mode='MANUAL',state='STOPPED')
        response=dict(state,ok=ok)
        if not cmd:response['token']=0
        r.fulfill(status=200 if ok else 409,content_type='application/json',body=json.dumps(response))
    page.route('http://robot.test/**',route)
    page.goto('http://robot.test/');page.wait_for_load_state('networkidle')
    expect(page.get_by_role('button',name='Start line following')).to_be_disabled()
    page.get_by_role('button',name='Take Wi-Fi control').click()
    expect(page.get_by_role('button',name='Start line following')).to_be_enabled()
    page.get_by_role('button',name='Start line following').click()
    expect(page.locator('#status')).to_contain_text('LINE_FOLLOW')
    page.wait_for_function("document.querySelector('#status').textContent.includes('Wi-Fi')")
    page.get_by_text('Tune line following',exact=True).click()
    page.get_by_label('Cruise mm/s',exact=True).fill('210')
    page.get_by_role('button',name='Apply speeds',exact=True).click()
    expect(page.locator('#message')).to_contain_text('Command accepted')
    assert 'LINESPEED 210 90 180 130' in commands
    page.get_by_role('button',name='STOP',exact=True).click()
    expect(page.locator('#message')).to_have_text('Stopped.')
    expect(page.get_by_role('button',name='Start line following')).to_be_disabled()
    # Page fits a narrow phone viewport and all script actions are error-free.
    assert page.evaluate('document.documentElement.scrollWidth <= innerWidth')
    assert not errors,errors
    page.screenshot(path='/tmp/cog300-wifi-page.png',full_page=True)
    # Lost request revokes the page's ownership instead of automatically claiming.
    page.get_by_role('button',name='Take Wi-Fi control').click()
    expect(page.get_by_role('button',name='Start line following')).to_be_enabled()
    page.route('http://robot.test/command',lambda r:r.abort() if r.request.post_data=='HEARTBEAT' else route(r))
    expect(page.locator('#message')).to_contain_text('Connection lost',timeout=4000)
    expect(page.get_by_role('button',name='Start line following')).to_be_disabled()
    assert commands.count('CLAIM')==2
    # An observer must not promise the active USB owner's lease will expire.
    state.update(owner='USB',token=0)
    page.get_by_role('button',name='STOP',exact=True).click()
    expect(page.locator('#message')).to_have_text('Stopped.')
    page.route('http://robot.test/command',lambda r:r.abort())
    page.get_by_role('button',name='STOP',exact=True).click()
    expect(page.locator('#message')).to_contain_text('Use the controlling connection')
    browser.close()
print('PASS Wi-Fi browser: claim/start/stop, tuning, loss, mobile layout, no script errors')
