#define ROBOT_ENABLE_STANDALONE 1
#include "firmware_fixture.h"
int main(){
 setupUnconfigured();assert(mode==MANUAL);assert(!startStandalone());
 hardware::Profile p;p.leftMmPerEdge=p.rightMmPerEdge=1;p.track=120;p.floorSensorOffset=90;p.leftSonarInstalled=true;p.front=90;p.rear=70;p.halfWidth=60;p.frontSonar=80;p.sideSonarX=30;p.sideSonarY=50;
 assert(configureHardware(p));serviceOdometry();tape(2);
 for(int n=0;n<3;++n)acceptReading(n,80,4640);
 assert(startStandalone());assert(mode==RACE&&automaticSetupActive());
 stopRobot();assert(!automaticSetupActive()&&mode==MANUAL);
 assert(startStandalone());
 float l=0,r=0,vl=0,vr=0;bool ran=false;
 for(int k=0;k<3000&&mode==RACE;++k){
  vl+=wall::limit(motorLeft*200/255.f-vl,-4.f,4.f);vr+=wall::limit(motorRight*190/255.f-vr,-4.f,4.f);
  l+=vl*.02f;r+=vr*.02f;leftTicks=(int32_t)lroundf(l);rightTicks=(int32_t)lroundf(r);nowMs+=20;
  serviceOdometry();for(int n=0;n<3;++n)acceptReading(n,80,4640);
  serviceNavigation();serviceMotor();
  if(!automaticSetupActive()){ran=true;break;}
 }
 assert(ran&&mode==RACE&&standaloneActive);assert(calibrationSaved());assert(lineTapeLevel==1);
 assert(wheelVelocity.config.valid()&&wallNavigator.config.valid()&&raceConfig.valid());
 stopRobot();assert(loadCalibration());assert(!automaticSetupActive());
 // Invalid compiled geometry must never restore a previously saved profile.
 assert(!configureHardware(hardware::Profile()));assert(!loadCalibration());assert(!startStandalone());
 assert(configureHardware(p));assert(loadCalibration());
 p.invertB=!p.invertB;configureHardware(p);assert(!loadCalibration());p.invertB=!p.invertB;configureHardware(p);assert(loadCalibration());
 auto old=hardwareFingerprint();p.track+=5;configureHardware(p);assert(hardwareFingerprint()!=old);assert(!loadCalibration());
}
