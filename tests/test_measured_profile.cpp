#include "firmware_fixture.h"
int main(){
 // Boot the actual compiled measurements, without replacing them with the
 // one-mm encoder fixtures used by older tests. No physical hardware is used.
 setup();assert(encodersEnabled()&&!leftSonarEnabled());
 assert(mode==MANUAL&&!motorDemandActive());assert(odometry.configured);
 assert(odometry.leftScale>5.10f&&odometry.leftScale<5.11f&&odometry.track==130);
 tape(2);acceptReading(0,80,4640);acceptReading(1,80,4640);
 command("HELLO");command("MODE CALIBRATE");assert(automaticSetupActive());
 float l=0,r=0,vl=0,vr=0;bool done=false;
 for(int k=0;k<3000;++k){
  vl+=wall::limit(motorLeft*200/255.f-vl,-4.f,4.f);vr+=wall::limit(motorRight*190/255.f-vr,-4.f,4.f);
  l+=vl*.02f;r+=vr*.02f;
  leftTicks=(int32_t)lroundf(l/odometry.leftScale);rightTicks=(int32_t)lroundf(r/odometry.rightScale);nowMs+=20;
  serviceOdometry();acceptReading(0,80,4640);acceptReading(1,80,4640);
  command("HEARTBEAT");serviceControlTimeout();serviceNavigation();serviceMotor();
  if(!automaticSetupActive()){done=true;break;}
 }
 assert(done&&calibrationSaved()&&mode==MANUAL&&!motorDemandActive());
 assert(raceConfig.sensorOffset==150&&wallNavigator.config.front==150&&wallNavigator.config.rear==70);
 command("MODE LINE");assert(mode==LINE);nowMs+=501;serviceControlTimeout();assert(mode==MANUAL&&!motorDemandActive());
}
