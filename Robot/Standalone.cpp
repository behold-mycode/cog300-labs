#include "Robot.h"
race::Config raceConfig;
race::Navigator raceNavigator;
bool standaloneActive=false;
static race::Button runButton;
static bool raceWallStarted=false,resumeWallGraph=false;
static uint32_t cueSince=0,quietSince=0;
static int lastCue=-1;
static bool usableTape(uint8_t mask){return mask==1||mask==2||mask==3||mask==4||mask==6;}
static void resetCue(){lastCue=-1;cueSince=quietSince=millis();}
void setupStandalone(){if(ROBOT_ENABLE_STANDALONE)pinMode(1,INPUT_PULLUP);}
bool startRace(){
 if(mapExportActive()){navigationState="BUSY_MAP_EXPORT";return false;}
 if(!hardwareFingerprint())return beginAutomaticSetup();
 if(!leftSonarEnabled()){navigationState="LEFT_SONAR_REQUIRED";return false;}
 if(lineTapeLevel<0||!prepareWallCalibration()||!raceConfig.valid())return beginAutomaticSetup();
 wallNavigator.clear();explorer.clear();resetOdometry();resetWallFeedback();raceWallStarted=false;resumeWallGraph=false;resetCue();
 if(!lineMask()){raceWallStarted=startWall();return raceWallStarted;}
 bool ok=raceNavigator.begin(raceConfig,wallNavigator.config,motionInput(),lineMask());
 navigationState=raceNavigator.reason;return ok;
}
bool startStandalone(){
 if(wirelessControlToken())return false;
 if(!ROBOT_ENABLE_STANDALONE){navigationState="USB_CONTROL_REQUIRED";return false;}
 if(mapExportActive()){navigationState="BUSY_MAP_EXPORT";return false;}
 stopRobot();suspendWireless();startNavigation(RACE);
 if(mode!=RACE)return false;
 controlSession=false;controlExpired=false;standaloneActive=true;setObservationTrace(false);return true;
}
void serviceButton(){
 if(!ROBOT_ENABLE_STANDALONE)return;
 auto e=runButton.update(digitalRead(1)==LOW,millis(),mode!=MANUAL||motorLeft||motorRight);
 if(e==race::Button::STOP){stopRobot();releaseWirelessControl();controlSession=false;controlExpired=true;}
 else if(e==race::Button::START)startStandalone();
}
void serviceRace(){
 if(automaticSetupActive()){serviceAutomaticSetup();return;}
 auto input=motionInput();uint8_t tape=lineMask();
 int cue=usableTape(tape)?1:tape==0?0:2;
 if(cue!=lastCue){lastCue=cue;cueSince=quietSince=input.now;}
 if(fmaxf(fabsf(input.leftSpeed),fabsf(input.rightSpeed))>=1)quietSince=input.now;
 bool settled=(uint32_t)(input.now-quietSince)>=250;
 bool stable=(uint32_t)(input.now-cueSince)>=200;
 if(raceWallStarted){
  // Finish a measured junction manoeuvre before changing guidance. Tape does
  // not cancel a pivot halfway or silently overwrite an unresolved map event.
  if(cue==1&&wallNavigator.state==wall::Navigator::CORRIDOR){
   stopMotor();pauseWall();navigationState="SOLVER_ACQUIRING_LINE";
   if(stable&&settled){
    if(raceNavigator.begin(raceConfig,wallNavigator.config,input,tape)){
     raceWallStarted=false;resumeWallGraph=true;resetWallFeedback();resetCue();
    }
   }
   return;
  }
  if(wallNavigator.paused&&wallNavigator.state==wall::Navigator::CORRIDOR)wallNavigator.resumeAfterLine(input);
  serviceWall();return;
 }
 auto d=raceNavigator.step(raceConfig,wallNavigator.config,input,tape);
 navigationState=raceNavigator.reason;driveVelocity(d);
 if(raceNavigator.wallStage()){
  resetWallFeedback();
  bool ok=resumeWallGraph?wallNavigator.resumeAfterLine(input):startWall();
  if(ok){raceWallStarted=true;resumeWallGraph=false;resetCue();navigationState="SOLVER_FOLLOWING_WALLS";}
  else{const char*why=resumeWallGraph?"SOLVER_MAP_RESUME_FAILED":navigationState;stopRobot();navigationState=why;}
 }
 if(raceNavigator.state==race::Navigator::FAULT){const char*why=raceNavigator.reason;stopRobot();navigationState=why;}
}

void reportRace(){
 Serial.print(F("RACENAV,"));Serial.print(raceConfig.valid()?1:0);Serial.print(',');Serial.print(calibrationSaved()?1:0);Serial.print(',');
 Serial.print(raceConfig.speed);Serial.print(',');Serial.print(raceConfig.sensorOffset);Serial.print(',');Serial.print(raceConfig.searchAngle);Serial.print(',');Serial.println((int)raceNavigator.state);
}
