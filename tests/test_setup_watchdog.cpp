#include "firmware_fixture.h"
int main(){
 setup();
 for(auto phase:{commissioning::Setup::RAMP,commissioning::Setup::MEASURE,commissioning::Setup::COAST}){
  configureHardware(syntheticHardware());tape(2);lineTapeLevel=-1;
  for(int n=0;n<3;++n)acceptReading(n,80,4640);
  command("HELLO");command("MODE RACE");assert(mode==RACE&&automaticSetupActive());
  float l=0,r=0,vl=0,vr=0;uint32_t ping=nowMs;int sensor=0;bool reached=false;
  unsigned char before[8192];memcpy(before,EEPROM.bytes,sizeof(before));
  for(int k=0;k<1000;++k){
   vl+=wall::limit(motorLeft*200/255.f-vl,-4.f,4.f);vr+=wall::limit(motorRight*190/255.f-vr,-4.f,4.f);
   l+=vl*.02f;r+=vr*.02f;leftTicks=(int32_t)lroundf(l);rightTicks=(int32_t)lroundf(r);nowMs+=20;
   serviceOdometry();if((uint32_t)(nowMs-ping)>=70){acceptReading(sensor,80,4640);sensor=(sensor+1)%3;ping=nowMs;}
   command("HEARTBEAT");serviceNavigation();serviceMotor();
   if(automaticSetup.state==phase){reached=true;break;}
  }
  assert(reached);nowMs+=501;serviceControlTimeout();assert(mode==MANUAL&&!automaticSetupActive());assert(!motorLeft&&!motorRight);
  for(int k=0;k<20;++k){nowMs+=100;serviceNavigation();serviceMotor();assert(!motorLeft&&!motorRight);}
  assert(!memcmp(before,EEPROM.bytes,sizeof(before)));
  command("RUN START");assert(mode==MANUAL); // Expired serial commands cannot revive setup.
 }
}
