#include "firmware_fixture.h"
int main(){
 setup();auto p=syntheticHardware();p.leftSonarInstalled=false;assert(configureHardware(p));serviceOdometry();tape(2);
 assert(!leftSonarEnabled());acceptReading(0,80,4640);acceptReading(1,80,4640);
 command("HELLO");command("MODE LINE");assert(mode==LINE&&automaticSetupActive());
 float l=0,r=0,vl=0,vr=0;bool ready=false;
 for(int k=0;k<3000;++k){
  vl+=wall::limit(motorLeft*200/255.f-vl,-4.f,4.f);vr+=wall::limit(motorRight*190/255.f-vr,-4.f,4.f);
  l+=vl*.02f;r+=vr*.02f;leftTicks=(int32_t)lroundf(l);rightTicks=(int32_t)lroundf(r);nowMs+=20;
  serviceOdometry();acceptReading(0,80,4640);acceptReading(1,80,4640);
  command("HEARTBEAT");serviceControlTimeout();serviceNavigation();serviceMotor();
  if(!automaticSetupActive()){ready=true;break;}
 }
 assert(ready&&mode==LINE&&calibrationSaved()&&!leftSonarEnabled());
 assert(raceConfig.sensorOffset==90);stopRobot();assert(loadCalibration());serviceOdometry();assert(!leftSonarEnabled());
 command("MODE RACE");assert(mode==MANUAL&&!automaticSetupActive());
 command("MODE OBJECT");assert(mode==OBJECT);tape(1);tick(20);assert(mode==MANUAL&&!motorLeft&&!motorRight);
 command("RUN START");assert(mode==MANUAL&&!lastCommandOk);
 tape(2);command("MODE LINE");nowMs+=501;serviceControlTimeout();assert(mode==MANUAL&&!motorDemandActive());
}
