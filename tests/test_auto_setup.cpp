#include "../Robot/AutoSetup.h"
#include <cassert>
#include <cstring>
int main(){
 hardware::Profile p;p.leftMmPerEdge=p.rightMmPerEdge=1;p.track=120;p.floorSensorOffset=90;p.leftSonarInstalled=true;p.front=90;p.rear=70;p.halfWidth=60;p.frontSonar=80;p.sideSonarX=30;p.sideSonarY=50;
 commissioning::Input i;for(auto&s:i.ranges){s.valid=true;s.mm=800;}i.rawTape=2;
 commissioning::Setup c;assert(c.begin(p,i));
 float l=0,r=0,vl=0,vr=0;wall::Powers out;
 for(int k=0;k<2000&&!c.done()&&!c.failed();++k){
  vl+=wall::limit(out.left*200/255.f-vl,-4.f,4.f);vr+=wall::limit(out.right*190/255.f-vr,-4.f,4.f);
  l+=vl*.02f;r+=vr*.02f;i.now+=20;i.left=l;i.right=r;
  for(auto&s:i.ranges){s.stamp=i.now;s.sequence++;}
  out=c.step(i);
 }
 assert(c.done());assert(c.result.tape==1);assert(c.result.fullLeft>180&&c.result.fullLeft<220);assert(c.result.fullRight>170&&c.result.fullRight<210);
 assert(c.result.deceleration>0&&c.result.deceleration<200);assert(l<300&&r<300);
 // Repeated data and no wheel counts cannot create a successful calibration.
 commissioning::Setup bad;i=commissioning::Input();i.rawTape=2;
 for(auto&s:i.ranges){s.valid=true;s.mm=800;}
 assert(bad.begin(p,i));for(int k=0;k<400&&!bad.failed();++k){i.now+=20;for(auto&s:i.ranges){s.stamp=i.now;s.sequence++;}bad.step(i);}
 assert(bad.failed());
 commissioning::Setup stale;assert(stale.begin(p,i));i.now+=500;assert(stale.step(i).left==0);assert(stale.failed());
 // The loop remains timely but a single sonar stops supplying fresh data.
 commissioning::Setup missing;i.now=0;for(auto&s:i.ranges){s.valid=true;s.stamp=0;s.mm=800;}
 assert(missing.begin(p,i));for(int k=0;k<20&&!missing.failed();++k){i.now+=20;i.ranges[0].stamp=i.ranges[1].stamp=i.now;missing.step(i);}
 assert(missing.failed()&&!strcmp(missing.reason,"SETUP_SENSOR_UNKNOWN"));
 commissioning::Setup stopped;assert(stopped.begin(p,i));stopped.cancel();assert(stopped.step(i).left==0&&!stopped.done());
}
