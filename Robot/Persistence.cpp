#include "Robot.h"
#include "CalibrationStore.h"
#include <EEPROM.h>
namespace {
struct Calibration {
 uint32_t hardwareId=0;
 Settings motors;
 int tape=-1,kind=0;bool invertLeft=false,invertRight=false,leftSonar=false;
 uint32_t interval=70;
 float leftScale=0,rightScale=0,track=0;
 wall::Config wall;
 wall::VelocityConfig velocity;
 race::Config race;
 bool valid()const{
  return hardwareMotorDirectionsMatch(motors.invertA,motors.invertB)&&motors.power>0&&motors.power<=255&&motors.minimum>=0&&motors.minimum<=motors.power&&
   tape>=0&&tape<=1&&kind>=1&&kind<=2&&interval>=25&&interval<=100&&
   isfinite(leftScale)&&leftScale>=.001f&&leftScale<=100&&isfinite(rightScale)&&rightScale>=.001f&&rightScale<=100&&
   isfinite(track)&&track>=10&&track<=1000&&wall.valid()&&velocity.valid()&&race.valid();
 }
};
bool saved=false;
}
bool calibrationSaved(){return saved;}
void clearSavedCalibrationStatus(){saved=false;}
bool saveCalibration(){
 if(!hardwareFingerprint()||mode!=MANUAL||motorLeft||motorRight||motorDemandActive()||!prepareWallCalibration(false))return false;
 Calibration c;c.hardwareId=hardwareFingerprint();c.motors=settings;c.tape=lineTapeLevel;c.kind=encoderKind();c.invertLeft=encoderInvertLeft();c.invertRight=encoderInvertRight();
 c.leftSonar=leftSonarEnabled();c.interval=sonarInterval();c.leftScale=odometry.leftScale;c.rightScale=odometry.rightScale;c.track=odometry.track;
 c.wall=wallNavigator.config;c.velocity=wheelVelocity.config;c.race=raceConfig;
 if(!c.valid())return false;
 saved=persistent::save(EEPROM,c);return saved;
}
bool loadCalibration(){
 if(!hardwareFingerprint()||mode!=MANUAL||motorLeft||motorRight||motorDemandActive()||explorer.graph.nodeCount)return false;
 Calibration c;if(!persistent::load(EEPROM,c)||!c.valid()||c.hardwareId!=hardwareFingerprint())return false;
 configureSonars(c.leftSonar,c.interval);configureEncoders(c.kind,c.invertLeft,c.invertRight);
 odometry.configure(c.leftScale,c.rightScale,c.track);
 settings=c.motors;lineTapeLevel=c.tape;wallNavigator.config=c.wall;wheelVelocity.config=c.velocity;raceConfig=c.race;
 resetWallFeedback();saved=true;return true;
}
