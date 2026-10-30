// Idealized kinematic regression, not hardware calibration: linear PWM speed,
// no slip/inertia, 14 cm wheel track, front/right offsets 8/6 cm, rectangular room.
// Uses real firmware and alternating 70 ms sensor samples to catch a corner
// transition that immediately steers right back into the same front wall.
#include <cassert>
#include <cmath>
#include <iostream>
#include "../Robot/Motor.cpp"
#include "../Robot/Sensors.cpp"
#include "../Robot/Navigation.cpp"
static double ray(double x, double y, double angle) {
  double dx=cos(angle), dy=sin(angle), distance=400;
  if(dx>0.00001) distance=fmin(distance,(150-x)/dx);
  if(dx<-.00001) distance=fmin(distance,-x/dx);
  if(dy>0.00001) distance=fmin(distance,(220-y)/dy);
  if(dy<-.00001) distance=fmin(distance,-y/dy);
  return distance;
}
static void corner(double speed) {
  stopRobot(); nowMs+=1000; startNavigation(WALL);
  double x=25,y=21,angle=0;
  int entries=0; bool wasTurning=false;
  for(int i=0;i<2500 && y<100;++i) {
    nowMs+=10;
    if(i%7==0) {
      int sensor=(i/7)%2;
      double beam=angle-(sensor?1.5707963267948966:0), offset=sensor?6:8;
      double distance=ray(x+offset*cos(beam),y+offset*sin(beam),beam);
      acceptReading(sensor,(int)distance,(uint32_t)(distance*58));
    }
    serviceNavigation(); serviceMotor();
    bool turning=!strcmp(navigationState,"TURNING_LEFT");
    if(turning && !wasTurning) ++entries;
    wasTurning=turning;
    // Initial wait for first right echo is allowed. A normal corner stays forward.
    if(i>=7) assert(motorLeft>0 && motorRight>0 && mode==WALL);
    double left=motorLeft/255.0*speed, right=motorRight/255.0*speed;
    angle+=(right-left)/14*.01;
    x+=(left+right)/2*cos(angle)*.01; y+=(left+right)/2*sin(angle)*.01;
    assert(x>8 && x<142 && y>8 && y<212);
  }
  std::cout << "speed=" << speed << " corner entries=" << entries << " heading=" << angle*180/3.141592653589793 << std::endl;
  assert(y>=100 && entries==1);
  assert(angle>1.22 && angle<1.92); // Roughly parallel after one continuous corner.
}
int main() {
  setupMotor();
  for(double speed : {20.,30.,40.}) corner(speed);
  std::cout << "PASS idealized forward-corner transition at three model speeds\n";
}
