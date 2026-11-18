#include "../Robot/Race.h"
#include <cassert>
int main(){
 wall::Config c;c.front=150;c.rear=70;c.halfWidth=80;c.frontSensor=150;c.sideX=45;c.sideY=65;c.margin=23;c.deceleration=200;c.reaction=.35f;c.cruise=50;c.turnSpeed=35;c.track=130;c.distanceTolerance=7.65f;c.angleTolerance=.12f;c.matchRadius=60;c.corridorWidth=400;
 race::Config cfg;cfg.speed=80;cfg.sensorOffset=150;
 wall::Input i;i.now=1000;race::Navigator line;
 assert(line.begin(cfg,c,i,2,false));
 auto d=line.step(cfg,c,i,2);assert(d.left>0&&d.right>0);
 d=line.step(cfg,c,i,4);assert(d.left<0&&d.right>0);
 line.step(cfg,c,i,0);i.now+=300;line.step(cfg,c,i,0);i.now+=20;
 d=line.step(cfg,c,i,0);assert(d.left!=0&&d.right!=0);
 race::Navigator maze;assert(maze.begin(cfg,c,i,2,true));
 d=maze.step(cfg,c,i,2);assert(d.left==0&&d.right==0);
}
