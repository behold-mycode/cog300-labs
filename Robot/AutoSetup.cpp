#include "Robot.h"
#include "AutoSetup.h"
static hardware::Profile activeHardware;
static commissioning::Setup automaticSetup;
static Mode setupDestination=RACE;
bool automaticSetupActive(){return automaticSetup.state>=commissioning::Setup::CHECK&&automaticSetup.state<=commissioning::Setup::COAST;}
void cancelAutomaticSetup(){automaticSetup.cancel();}
float lineSensorOffset(){return activeHardware.floorSensorOffset;}
uint32_t hardwareFingerprint(){return activeHardware.valid()?activeHardware.fingerprint():0;}
bool hardwareMotorDirectionsMatch(bool a,bool b){return activeHardware.valid()&&a==activeHardware.invertA&&b==activeHardware.invertB;}
bool configureHardware(const hardware::Profile&p){
 stopRobot();clearSavedCalibrationStatus();activeHardware=p;
 if(!p.valid()){navigationState="HARDWARE_DIMENSIONS_REQUIRED";return false;}
 settings.invertA=p.invertA;settings.invertB=p.invertB;
 configureSonars(p.leftSonarInstalled,70);configureEncoders(1,false,false);
 odometry.configure(p.leftMmPerEdge,p.rightMmPerEdge,p.track);serviceOdometry();
 navigationState="READY_FOR_FIRST_START_CHECK";return true;
}
static commissioning::Input setupInput(){
 commissioning::Input i;i.now=millis();i.left=odometry.pose.left;i.right=odometry.pose.right;
 i.rawTape=(digitalRead(LINE_LEFT_PIN)==HIGH?4:0)|(digitalRead(LINE_CENTER_PIN)==HIGH?2:0)|(digitalRead(LINE_RIGHT_PIN)==HIGH?1:0);
 for(int n=0;n<3;++n){auto s=sensorReading(n);i.ranges[n]={s.cm*10,s.sampledMs,s.sequence,s.status==RANGE_VALID,sensorPose(n),s.status==RANGE_NO_RETURN};}
 return i;
}
bool beginAutomaticSetup(Mode destination){
 setupDestination=destination;
 if(!activeHardware.valid()){navigationState="HARDWARE_DIMENSIONS_REQUIRED";return false;}
 if(!encodersEnabled()||!odometry.configured){navigationState="ENCODERS_REQUIRED";return false;}
 serviceOdometry();bool ok=automaticSetup.begin(activeHardware,setupInput());navigationState=automaticSetup.reason;return ok;
}
void serviceAutomaticSetup(){
 auto p=automaticSetup.step(setupInput());navigationState=automaticSetup.reason;requestMotor(p.left,p.right);
 if(automaticSetup.failed()){const char*why=automaticSetup.reason;stopRobot();navigationState=why;return;}
 if(!automaticSetup.done())return;
 stopMotor();const auto&r=automaticSetup.result;const auto&h=activeHardware;
 wall::Config c;c.front=h.front;c.rear=h.rear;c.halfWidth=h.halfWidth;c.frontSensor=h.frontSonar;c.sideX=h.sideSonarX;c.sideY=h.sideSonarY;
 c.track=h.track;c.distanceTolerance=fmaxf(h.leftMmPerEdge,h.rightMmPerEdge)*1.5f;c.angleTolerance=fmaxf(.025f,2*c.distanceTolerance/c.track);
 c.margin=fmaxf(20,3*c.distanceTolerance);c.matchRadius=fmaxf(60,4*c.distanceTolerance);
 c.deceleration=r.deceleration;c.reaction=.35f;c.cruise=r.cruise;c.turnSpeed=fminf(40,r.cruise*.6f);
 c.corridorWidth=fmaxf(r.corridorWidth,2*(c.halfWidth+c.margin)+20);
 wallNavigator.config=c;wheelVelocity.config=wall::VelocityConfig();wheelVelocity.config.fullLeft=r.fullLeft;wheelVelocity.config.fullRight=r.fullRight;
 wheelVelocity.config.acceleration=fminf(150,r.deceleration);
 lineTapeLevel=r.tape;raceConfig.speed=r.cruise;raceConfig.sensorOffset=h.floorSensorOffset;
 // Flash writes occur only after measured coast and with bridge disabled.
 mode=MANUAL;bool saved=saveCalibration();mode=setupDestination;
 if(!saved){stopRobot();navigationState="CALIBRATION_SAVE_FAILED";return;}
 automaticSetup.cancel();resetWallFeedback();
 if(setupDestination==CALIBRATE){stopRobot();navigationState="CALIBRATION_COMPLETE";return;}
 bool wasStandalone=standaloneActive;
 startNavigation(setupDestination);
 standaloneActive=wasStandalone&&mode!=MANUAL;
}
