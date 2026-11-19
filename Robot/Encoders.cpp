#include "Robot.h"
#include "Odometry.h"

maze::Odometry odometry;
uint32_t odometryEpoch=0;
static volatile uint32_t leftTicks=0,rightTicks=0;
static volatile int leftDirection=1,rightDirection=1;
static bool encEnabled=false,encQuadrature=false;
static int encInvertL=1,encInvertR=1;
// Opt-in only. R4 IRQ channels: D12=5, A4=1; sonar A1=6/A3=2.
// B channels are sampled on A changes (x2 decoding), not attached to IRQs.
static const uint8_t ENC_LA=12,ENC_LB=13,ENC_RA=A4,ENC_RB=A5;
static void leftEncoderChanged() {
  int direction=encQuadrature ? (digitalRead(ENC_LA)==digitalRead(ENC_LB)?1:-1)*encInvertL : leftDirection;
  leftTicks+=direction;
}
static void rightEncoderChanged() {
  int direction=encQuadrature ? (digitalRead(ENC_RA)==digitalRead(ENC_RB)?1:-1)*encInvertR : rightDirection;
  rightTicks+=direction;
}
bool encodersEnabled() { return encEnabled; }
bool configureEncoders(int kind,bool invertL,bool invertR) {
  if(kind<0||kind>2) return false;
  stopRobot();resetWallCalibration();
  if(encEnabled) { detachInterrupt(digitalPinToInterrupt(ENC_LA));detachInterrupt(digitalPinToInterrupt(ENC_RA)); }
  ++odometryEpoch;encEnabled=false;odometry.sampled=false;odometry.configured=false;explorer.geometry=maze::Geometry();explorer.dynamics=maze::Dynamics();explorer.world.reset();
  if(!kind) return true;
  encQuadrature=kind==2;encInvertL=invertL?-1:1;encInvertR=invertR?-1:1;
  // No pullups: actual module voltage/output type must be checked before enabling.
  pinMode(ENC_LA,INPUT);pinMode(ENC_RA,INPUT);
  if(encQuadrature) { pinMode(ENC_LB,INPUT);pinMode(ENC_RB,INPUT); }
  noInterrupts();leftTicks=rightTicks=0;interrupts();
  attachInterrupt(digitalPinToInterrupt(ENC_LA),leftEncoderChanged,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RA),rightEncoderChanged,CHANGE);
  encEnabled=true;return true;
}
void updateEncoderDirections(int left,int right) {
  // Called before enabling the bridge; retain direction while coasting.
  if(left) leftDirection=left>0?1:-1;
  if(right) rightDirection=right>0?1:-1;
}
void serviceOdometry() {
  if(!encEnabled) return;
  noInterrupts();int32_t l=(int32_t)leftTicks,r=(int32_t)rightTicks;interrupts();
  odometry.update(l,r,millis());
}
void resetOdometry() {
  ++odometryEpoch;
  noInterrupts();int32_t l=(int32_t)leftTicks,r=(int32_t)rightTicks;interrupts();
  odometry.reset(l,r,millis());
}

void reportEncoders() {
  noInterrupts();int32_t l=(int32_t)leftTicks,r=(int32_t)rightTicks;interrupts();
  Serial.print(F("COUNTS,"));Serial.print(l);Serial.print(',');Serial.print(r);Serial.print(',');
  Serial.println(encEnabled?(encQuadrature?2:1):0);
}

int encoderKind(){return encEnabled?(encQuadrature?2:1):0;}
bool encoderInvertLeft(){return encInvertL<0;}bool encoderInvertRight(){return encInvertR<0;}
