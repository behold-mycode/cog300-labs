#include "../Robot/Race.h"
#include <cassert>
#include <initializer_list>

int main(){
 for(int side:{1,-1}){
 wall::Config c;c.front=150;c.rear=70;c.halfWidth=80;c.frontSensor=150;c.sideX=45;c.sideY=65;c.margin=23;c.deceleration=200;c.reaction=.35f;c.cruise=50;c.turnSpeed=35;c.track=130;c.distanceTolerance=7.65f;c.angleTolerance=.12f;c.matchRadius=60;c.corridorWidth=400;
 race::Config cfg;cfg.speed=80;cfg.sensorOffset=150;wall::Input i;race::Navigator nav;
 assert(nav.begin(cfg,c,i,2,false));
 i.now=20;i.pose.left=i.pose.right=20;nav.step(cfg,c,i,side==1?4:1);
 // The robot travels beyond the last tape sample before coming to rest.
 i.now=40;i.pose.left=i.pose.right=60;nav.step(cfg,c,i,0);
 i.now=340;i.pose.left=i.pose.right=80;nav.step(cfg,c,i,0);
 i.now=360;auto d=nav.step(cfg,c,i,0);assert(d.left<0&&d.right<0);
 i.now=380;i.pose.left=20;i.pose.right=40;d=nav.step(cfg,c,i,0);assert(d.left==0&&d.right<0);
 i.now=400;i.pose.right=20;d=nav.step(cfg,c,i,0);assert(d.left==0&&d.right==0);
 i.now=700;nav.step(cfg,c,i,0);i.now=720;d=nav.step(cfg,c,i,0);
 assert(d.left*side<0&&d.right*side>0); // Search toward remembered tape side.
 nav.stop();assert(nav.begin(cfg,c,i,2,false));
 i.now=740;nav.step(cfg,c,i,side==1?4:1);
 i.pose.left-=30*side;i.pose.right+=30*side;i.now=760;nav.step(cfg,c,i,0);
 i.now=1060;nav.step(cfg,c,i,0);i.now=1080;d=nav.step(cfg,c,i,0);
 assert(d.left*side>0&&d.right*side<0); // Undo pivot overshoot before searching.
 nav.stop();d=nav.step(cfg,c,i,0);assert(d.left==0&&d.right==0);
 }
}
