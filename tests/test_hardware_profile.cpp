#include "../Robot/HardwareProfile.h"
#include <cassert>
int main(){
 hardware::Profile p;assert(!p.valid());assert(!hardware::CAR.valid());
 p.leftMmPerEdge=1;p.rightMmPerEdge=1;p.track=120;
 p.floorSensorOffset=90;p.leftSonarInstalled=true;p.front=90;p.rear=70;p.halfWidth=60;p.frontSonar=80;p.sideSonarX=30;p.sideSonarY=50;
 assert(p.valid());auto polarityId=p.fingerprint();p.invertA=!p.invertA;assert(p.fingerprint()!=polarityId);p.invertA=!p.invertA;auto id=p.fingerprint();p.track=125;assert(p.fingerprint()!=id);
 p.frontSonar=100;assert(!p.valid());p.frontSonar=80;p.leftMmPerEdge=100;assert(!p.valid());
}
