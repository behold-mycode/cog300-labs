#!/usr/bin/env python3
"""Mac desktop controls. No global keyboard hook; only the focused window drives."""
import argparse
from collections import deque
from datetime import datetime
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import serial
from serial.tools import list_ports
from controller_core import Controller


class RobotWindow:
    def __init__(self, root, demo=False):
        self.root, self.demo = root, demo
        self.port = None
        self.buffer = b''
        self.demo_due = None
        self.history = deque(maxlen=5000)
        self.releases = {}
        self.core = Controller(self.send, time.monotonic, self.log)
        root.title('COG300 Robot Controller' + (' — DEMO (no robot)' if demo else ''))
        root.geometry('850x780'); root.minsize(790, 740)
        self.status = tk.StringVar(value='Disconnected')
        self.port_name = tk.StringVar()
        self.speed = tk.IntVar(value=255)
        self.duration = tk.IntVar(value=500)
        self.left_trim = tk.IntVar(value=100); self.right_trim = tk.IntVar(value=100)
        self.left_invert = tk.BooleanVar(value=False); self.right_invert = tk.BooleanVar(value=True)
        self.build()
        root.bind('<KeyPress>', self.key_down)
        root.bind('<KeyRelease>', self.key_up)
        root.bind('<FocusOut>', lambda event: root.after_idle(self.check_focus))
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.refresh_ports()
        self.log('Connect after uploading firmware 2. Close Arduino Serial Monitor first.')
        if demo: self.log('DEMO MODE: all responses are simulated; no serial port is opened.')
        root.after(25, self.poll)

    def build(self):
        pad = ttk.Frame(self.root, padding=18); pad.pack(fill='both', expand=True)
        ttk.Label(pad, text='Robot control', font=('Helvetica', 24, 'bold')).pack(anchor='w')
        ttk.Label(pad, text='Hold arrows to drive · release to stop · Space / Esc stops everything').pack(anchor='w', pady=(3,12))
        row = ttk.Frame(pad); row.pack(fill='x')
        self.port_box = ttk.Combobox(row, textvariable=self.port_name, state='readonly', width=32)
        self.port_box.pack(side='left')
        ttk.Button(row, text='Refresh', command=self.refresh_ports).pack(side='left', padx=5)
        self.connect_button = ttk.Button(row, text='Connect', command=self.toggle_connection)
        self.connect_button.pack(side='left')
        ttk.Button(row, text='Save log', command=self.save_log).pack(side='right')
        ttk.Label(pad, textvariable=self.status, foreground='#16517a', wraplength=770).pack(anchor='w', pady=10)
        settings = ttk.Frame(pad); settings.pack(fill='x')
        ttk.Label(settings, text='PWM').pack(side='left')
        self.speed_box = ttk.Spinbox(settings, from_=0, to=255, textvariable=self.speed, width=6)
        self.speed_box.pack(side='left', padx=(6,20))
        ttk.Label(settings, text='Test duration (ms)').pack(side='left')
        self.duration_box = ttk.Spinbox(settings, from_=100, to=5000, increment=100, textvariable=self.duration, width=7)
        self.duration_box.pack(side='left', padx=6)
        ttk.Button(settings, text='Enable controls', command=self.enable).pack(side='right')
        self.drive_pad = ttk.LabelFrame(pad, text='Driving — arrows / on-screen buttons', padding=12)
        self.drive_pad.pack(fill='x', pady=12)
        self.drive_pad.configure(takefocus=True)
        grid = ttk.Frame(self.drive_pad); grid.pack()
        for label, key, row, column in [('↑ Forward','Up',0,1), ('← Left','Left',1,0),
                                         ('↓ Reverse','Down',1,1), ('Right →','Right',1,2)]:
            button = ttk.Button(grid, text=label, width=17)
            button.grid(row=row, column=column, padx=4, pady=3)
            button.bind('<ButtonPress-1>', lambda event, k=key: self.mouse_press(k))
            button.bind('<ButtonRelease-1>', lambda event, k=key: self.core.release(k))
            button.bind('<Leave>', lambda event, k=key: self.core.release(k))
        tk.Button(grid, text='STOP  ·  Space', bg='#ffffff', fg='#a01825',
                  font=('Helvetica', 13, 'bold'), command=self.stop).grid(row=2,column=0,columnspan=3,sticky='ew',pady=8)
        ttk.Label(self.drive_pad, text='Left/right alone turn in place. Up + left/right makes a moving turn.').pack()
        tests = ttk.LabelFrame(pad, text='Repeatable lab tests — buttons or F1–F5 (Fn may be needed on Mac)', padding=10)
        tests.pack(fill='x')
        for key, name in self.core.HOTKEYS.items():
            ttk.Button(tests, text=f'{key}  {name}', command=lambda n=name: self.start_test(n)).pack(side='left',expand=True,fill='x',padx=3)
        calib = ttk.LabelFrame(pad, text='Motor calibration — start at 100% on each wheel', padding=10)
        calib.pack(fill='x', pady=12)
        for row, name, trim, inv in [(0,'Left',self.left_trim,self.left_invert),(1,'Right',self.right_trim,self.right_invert)]:
            ttk.Label(calib,text=name+' trim %').grid(row=row,column=0,sticky='w',padx=4,pady=3)
            ttk.Spinbox(calib,from_=1,to=200,textvariable=trim,width=6).grid(row=row,column=1,padx=4)
            ttk.Checkbutton(calib,text='Reverse direction',variable=inv).grid(row=row,column=2,padx=15)
        ttk.Button(calib,text='Apply (stops robot)',command=self.apply_settings).grid(row=0,column=3,rowspan=2,padx=12)
        ttk.Label(calib,text='Settings last until reset. PWM caps at 255; trim cannot repair a loose wire.').grid(row=2,column=0,columnspan=4,sticky='w',pady=(6,0))
        ttk.Label(pad,text='Log — completion means commands finished; observe the wheels to judge PASS / FAIL.').pack(anchor='w')
        self.logbox = tk.Text(pad,height=9,font=('Menlo',10),state='disabled',wrap='word')
        self.logbox.pack(fill='both',expand=True,pady=(5,0))

    def log(self, message):
        line = datetime.now().strftime('%H:%M:%S.%f')[:-3] + ' ' + message
        self.history.append(line)
        if hasattr(self,'logbox'):
            self.logbox.configure(state='normal'); self.logbox.insert('end',line+'\n')
            if int(self.logbox.index('end-1c').split('.')[0]) > 600:
                self.logbox.delete('1.0','101.0')
            self.logbox.see('end'); self.logbox.configure(state='disabled')

    def send(self, command):
        if command != 'HEARTBEAT' and not command.startswith('DRIVE '): self.log('> '+command)
        if self.demo:
            if command == 'HELLO': self.root.after(5,lambda:self.core.receive('ROBOT_READY 2'))
            elif command.startswith('CONFIG '): self.root.after(5,lambda:self.core.receive('CONFIG_OK'))
            elif command == 'HEARTBEAT': self.root.after(5,lambda:self.core.receive('ALIVE'))
            elif command.startswith('TEST '):
                _, request, left, right, ms = command.split()
                self.demo_due = (time.monotonic()+.25+int(ms)/1000,request)
                self.log(f'DEMO only: requested L={left} R={right}')
            elif command == 'STOP': self.demo_due = None
            return
        if self.port:
            data = (command+'\n').encode('ascii')
            if self.port.write(data) != len(data): raise serial.SerialTimeoutException('Incomplete serial write')

    def refresh_ports(self):
        ports = ['DEMO — no hardware'] if self.demo else [p.device for p in list_ports.comports() if 'Bluetooth' not in p.device]
        self.port_box['values'] = ports
        if self.port_name.get() not in ports:
            preferred = next((p for p in ports if 'usbmodem' in p),ports[0] if ports else '')
            self.port_name.set(preferred)

    def toggle_connection(self):
        if self.core.connected:
            self.core.disconnect(); self.close_port(); return
        if not self.port_name.get(): messagebox.showinfo('No board','Connect the Arduino by USB, then Refresh.'); return
        try:
            if not self.demo:
                self.port = serial.Serial(self.port_name.get(),9600,timeout=0,write_timeout=.05,exclusive=True)
                self.port.reset_input_buffer()
            self.buffer = b''
            self.core.trim = (self.left_trim.get(),self.right_trim.get(),self.left_invert.get(),self.right_invert.get())
            if not all(1 <= v <= 200 for v in self.core.trim[:2]): raise ValueError('Trim must be 1–200%.')
            self.core.connect()
        except (serial.SerialException,OSError,ValueError,tk.TclError) as error:
            self.io_error(error)

    def close_port(self):
        if self.port:
            self.port.close(); self.port = None

    def io_error(self, error):
        self.close_port(); self.core.connected = False
        self.core.fail('Connection error — reconnect'); self.log(str(error))

    def read_values(self):
        speed, duration = self.speed.get(), self.duration.get()
        if not 0 <= speed <= 255 or not 100 <= duration <= 5000:
            raise ValueError('PWM must be 0–255 and test duration 100–5000 ms.')
        self.core.speed = speed; self.core.duration = duration

    def enable(self):
        try:
            self.read_values(); self.core.enable(); self.drive_pad.focus_set()
        except (ValueError,tk.TclError) as error: messagebox.showerror('Check settings',str(error))

    def stop(self):
        try: self.core.stop()
        except (serial.SerialException,OSError) as error: self.io_error(error)

    def apply_settings(self):
        try:
            self.read_values()
            self.core.configure(self.left_trim.get(),self.right_trim.get(),self.left_invert.get(),self.right_invert.get())
        except (ValueError,tk.TclError) as error: messagebox.showerror('Check settings',str(error))
        except (serial.SerialException,OSError) as error: self.io_error(error)

    def start_test(self, name):
        try:
            self.read_values(); self.core.start_test(name); self.drive_pad.focus_set()
        except (ValueError,tk.TclError) as error: messagebox.showerror('Check settings',str(error))
        except (serial.SerialException,OSError) as error: self.io_error(error)

    def mouse_press(self,key):
        self.drive_pad.focus_set()
        try: self.read_values(); self.core.press(key)
        except (ValueError,tk.TclError): self.stop()

    def key_down(self,event):
        if event.keysym in ('space','Escape'):
            self.stop(); return 'break'
        # Never treat editing fields, dropdown navigation, or command shortcuts as drive inputs.
        if isinstance(event.widget,(tk.Entry,ttk.Entry,ttk.Spinbox,ttk.Combobox,tk.Text)):
            return
        if event.state & (0x4 | 0x8 | 0x10): return
        key = event.keysym
        if key in self.releases:
            self.root.after_cancel(self.releases.pop(key))
        if key in self.core.ARROWS or key in self.core.HOTKEYS:
            try: self.read_values(); self.core.press(key)
            except (ValueError,tk.TclError): self.stop()
            except (serial.SerialException,OSError) as error: self.io_error(error)
            return 'break'

    def key_up(self,event):
        key = event.keysym
        # Defer one event-loop turn to coalesce platform auto-repeat release/press pairs.
        if key in self.releases: self.root.after_cancel(self.releases.pop(key))
        def release():
            self.releases.pop(key,None)
            try: self.core.release(key)
            except (serial.SerialException,OSError) as error: self.io_error(error)
        self.releases[key] = self.root.after_idle(release)

    def check_focus(self):
        focus = self.root.focus_get()
        if focus is None or isinstance(focus,(ttk.Entry,ttk.Spinbox,ttk.Combobox,tk.Entry)):
            if self.core.enabled:
                try: self.core.disable()
                except (serial.SerialException,OSError) as error: self.io_error(error)

    def poll(self):
        try:
            if self.port:
                self.buffer += self.port.read(min(self.port.in_waiting,4096))
                if len(self.buffer)>16384: raise ValueError('Oversized serial response')
                while b'\n' in self.buffer:
                    line,self.buffer = self.buffer.split(b'\n',1)
                    self.core.receive(line.decode('utf8',errors='replace').strip())
            if self.demo_due and time.monotonic()>=self.demo_due[0]:
                request=self.demo_due[1]; self.demo_due=None
                self.core.receive('TEST_DONE '+request)
            self.core.tick()
        except (serial.SerialException,OSError,ValueError) as error: self.io_error(error)
        self.status.set(self.core.phase + ('  [DEMO — no robot]' if self.demo else ''))
        self.connect_button.configure(text='Disconnect' if self.core.connected else 'Connect')
        self.root.after(25,self.poll)

    def save_log(self):
        try: self.core.disable()
        except (serial.SerialException,OSError) as error: self.io_error(error)
        path=filedialog.asksaveasfilename(defaultextension='.txt',initialfile=datetime.now().strftime('robot-test-%Y%m%d-%H%M%S.txt'))
        if path:
            with open(path,'w') as file: file.write('\n'.join(self.history)+'\n')

    def close(self):
        try: self.core.disconnect()
        except (serial.SerialException,OSError): pass
        finally: self.close_port(); self.root.destroy()


if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--demo',action='store_true')
    args=parser.parse_args()
    root=tk.Tk(); app=RobotWindow(root,args.demo); root.mainloop()
