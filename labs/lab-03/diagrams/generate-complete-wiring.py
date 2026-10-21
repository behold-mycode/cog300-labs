from pathlib import Path
from html import escape
out=Path(__file__).resolve().parent
for bread in [False,True]:
 a=['<svg xmlns="http://www.w3.org/2000/svg" width="2200" height="2200" viewBox="0 0 2200 2200"><rect width="2200" height="2200" fill="#f4f7fa"/><style>text{font-family:Arial,sans-serif;fill:#183044}.title{font-size:38px;font-weight:bold}.head{font-size:27px;font-weight:bold}.body{font-size:23px}.small{font-size:20px}</style>']
 def t(x,y,s,c='body'):a.append(f'<text x="{x}" y="{y}" class="{c}">{escape(s)}</text>')
 def box(x,y,w,h,c='#fff'):a.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" fill="{c}" stroke="#afc2ce" stroke-width="2"/>')
 def wire(points,c='#344554',dash=False):a.append(f'<polyline points="{points}" fill="none" stroke="{c}" stroke-width="4"'+(' stroke-dasharray="10 8"' if dash else '')+'/>')
 def dot(x,y,c):a.append(f'<circle cx="{x}" cy="{y}" r="6" fill="{c}"/>')
 red='#bd481f';black='#344554';yellow='#a16b00';purple='#7752ad'
 t(55,60,'COG300 • Complete robot wiring'+(' — two breadboards' if bread else ''),'title')
 t(55,102,'UNO R4 WiFi • L298N • two HC-SR04 sensors • two motors • green status LED')
 t(55,138,'Solid connections: current USB bench arrangement. Battery is NOT connected. Match terminal labels, not drawn board positions.','small')
 box(70,185,360,1205,'#e5f2f1');t(95,230,'UNO R4 WiFi','head');t(95,265,'Main 5 V GPIO headers','small')
 box(70,1450,360,100);t(95,1490,'Laptop USB','head');wire('250,1450 250,1390','#227b98');t(270,1425,'USB-C','small')
 # two sensor modules: distinct 4-wire harnesses
 for y,title,pins in [(190,'FRONT • faces forward',['5V','A0','A1','GND']),(510,'RIGHT • faces right →',['5V','A2','A3','GND'])]:
  box(1100,y,1030,285);t(1125,y+40,title,'head')
  t(1125,y+73,'Separate breadboard; illustrative row numbers' if bread else 'HC-SR04 • standard 5 V module','small')
  for i,(pin,term,col) in enumerate(zip(pins,['VCC','TRIG','ECHO','GND'],[red,yellow,purple,black])):
   yy=y+110+i*43
   t(335,yy+8,pin);wire(f'430,{yy} 1100,{yy}',col)
   t(610,yy-10,('5 V supply' if i==0 else 'Trigger →' if i==1 else '← Echo' if i==2 else 'Ground'), 'small')
   if bread:
    # one electrically continuous group of five per pin, separate rows
    wire(f'1100,{yy} 1700,{yy}',col)
    for xx in [1300,1340,1380,1420,1460]:
     a.append(f'<circle cx="{xx}" cy="{yy}" r="6" fill="white" stroke="#617785" stroke-width="2"/>')
    t(1130,yy-9,f'Row {10+i}', 'small');t(1510,yy-9,term)
    wire(f'1620,{yy} 1700,{yy}',col)
   else:
    t(1130,yy+7,term)
  if bread:
   box(1700,y+90,400,175,'#e5f2f1');t(1730,y+125,'HC-SR04','head');t(1730,y+165,'Pins on FOUR','small');t(1730,y+195,'different rows','small');t(1730,y+235,'a–e side only','small')
 # LEDs
 for y,name,pin,col in [(850,'GREEN LED','D11','#16834a')]:
  t(325,y+8,pin);wire(f'430,{y} 1120,{y}',col);box(1120,y-15,120,30);t(1135,y-25,'1 kΩ','small');wire(f'1240,{y} 1370,{y}',col)
  a.append(f'<circle cx="1400" cy="{y}" r="30" fill="{col}"/>');t(1450,y+7,name,'head');t(1300,y+48,'anode (+)     cathode (−)','small')
  wire(f'1430,{y} 1430,{y+65} 1840,{y+65}',black);t(1850,y+72,'GND rail','small')
 t(1100,970,'Green ON = front clear; OFF = blocked / unknown.','small')
 t(1100,1005,'Sensors A0–A3 • motor header D5–D10 • green LED D11.','small')
 # motor driver with all signal paths
 box(1100,1045,570,345,'#fcece7');t(1130,1085,'L298N driver','head')
 for i,(pin,term) in enumerate([('D5','ENA'),('D6','IN1'),('D7','IN2'),('D8','IN3'),('D9','IN4'),('D10','ENB')]):
  yy=1120+i*44;t(325,yy+7,pin);wire(f'430,{yy} 1100,{yy}','#227b98');t(1130,yy+7,term)
 for y,name,outs in [(1150,'LEFT MOTOR',['OUT1','OUT2']),(1290,'RIGHT MOTOR',['OUT3','OUT4'])]:
  box(1880,y-45,260,100,'#e7edf5');t(1900,y,name,'head')
  for i,term in enumerate(outs):
   yy=y-15+i*30;t(1540,yy+7,term,'small');wire(f'1670,{yy} 1880,{yy}', '#227b98')
 t(1110,1425,'ENA + ENB caps: REMOVED. 5V-EN: REMOVED.','small')
 # power net blocks complete explicit fanout (net labels match sensor GPIO panel)
 box(500,1465,1640,235);t(525,1505,'POWER DISTRIBUTION — all matching rail names are electrically joined','head')
 wire('545,1560 2060,1560',red)
 for x,label in [(545,'Uno 5V'),(850,'Front VCC'),(1150,'Right VCC*'),(1480,'L298N logic 5V'),(1830,'L298N VS / +12V**')]:
  dot(x,1560,red);wire(f'{x},1560 {x},1530',red);t(x-20,1538,label,'small')
 wire('545,1655 2060,1655',black)
 for x,label in [(545,'Uno GND'),(850,'Front GND'),(1150,'Right GND'),(1480,'L298N GND'),(1830,'Green LED cathode')]:
  dot(x,1655,black);t(x-20,1640,label,'small')
 t(55,1740,'* Right VCC: leave disconnected until dual-sensor firmware is installed. Current V2 measures front only.','body')
 t(55,1780,'** VS is currently fed from Uno 5V in the course bench setup. Remove that wire BEFORE connecting a rated motor battery.','body')
 t(55,1820,'Future AA battery: specs pending. Do not connect it yet. Sensors and driver logic must remain on regulated 5 V.','body')
 t(55,1860,('Breadboard: each shown row is five joined holes; different rows are isolated. No sensor pins share a row. See guide for LED rows.' if bread else 'Rail labels denote the same electrical node. Motor pairs go across OUT terminals, never to GND. Power off before rewiring.'),'small')
 a.append('</svg>');(out/('robot-complete-breadboards.svg' if bread else 'robot-complete-wiring.svg')).write_text('\n'.join(a))
