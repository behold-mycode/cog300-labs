#include "RadarSensor.h"
#if defined(ARDUINO_UNOWIFIR4)
#include <pwm.h>
static PwmOut radarPwm(1);
#endif

// Stagger pings; never block waiting for an echo. Third sonar is opt-in.
const uint8_t FRONT_SENSOR = 0, RIGHT_SENSOR = 1;
const uint8_t triggerPins[] = {TRIG_PIN, RIGHT_TRIG_PIN, 0};
const uint8_t echoPins[] = {ECHO_PIN, RIGHT_ECHO_PIN, 11};
static uint8_t sonarCount=2;
static uint32_t pingInterval=PING_MS;
static maze::Pose pingPose;
static maze::Pose samplePoses[3];
struct DistanceSample {
  bool valid = false;
  RangeStatus status = RANGE_INVALID;
  int cm = -1;
  uint32_t echoUs = 0;
  uint32_t sampledMs = 0;
  uint32_t sequence = 0;
};
DistanceSample samples[3];
uint8_t nextSensor = FRONT_SENSOR;
volatile uint8_t activeSensor = FRONT_SENSOR;
uint32_t lastPingMs = 0, pingStartUs = 0, pingStartMs=0;
bool pingPending = false;
volatile bool echoArmed = false, echoRisen = false, echoReady = false;
volatile uint32_t echoStartUs = 0, echoWidthUs = 0;

// A generation marks each slew, including a return to the same angle while a
// ping is in flight. Only a trigger from the current generation can publish.
static bool radarEnabled = false, radarPwmOk = false;
static int commandedBearing = 0;
static uint32_t aimGeneration = 0, pingGeneration = 0, sampleGeneration = 0;
static uint32_t aimChangedMs = 0, settleMs = 0;

static int centrePulse = RADAR_CENTER_US;
static int leftPulse = RADAR_CENTER_US + RADAR_LEFT_SIGN * 60 * RADAR_US_PER_DEGREE;
static int rightPulse = RADAR_CENTER_US - RADAR_LEFT_SIGN * 60 * RADAR_US_PER_DEGREE;

bool radarServoCalibration(int rightUs,int centreUs,int leftUs){
  int rightDelta=rightUs-centreUs,leftDelta=leftUs-centreUs;
  if(rightUs<800||rightUs>2200||centreUs<800||centreUs>2200||leftUs<800||leftUs>2200||
     abs(rightDelta)<100||abs(leftDelta)<100||((rightDelta<0)==(leftDelta<0)))return false;
  centrePulse=centreUs;leftPulse=leftUs;rightPulse=rightUs;
  if(!radarEnabled)radarEnable();
  else{
    commandedBearing=0;++aimGeneration;aimChangedMs=millis();settleMs=500;
#if defined(ARDUINO_UNOWIFIR4) && !ROBOT_ENABLE_STANDALONE
    radarPwmOk=radarPwm.pulseWidth_us(centrePulse);
#endif
  }
  return radarReady();
}

bool radarReady() { return radarPwmOk; }
int radarBearing() { return commandedBearing; }

void radarEnable() {
  if (radarEnabled) return;
  radarEnabled = true;
  ++aimGeneration;
  aimChangedMs = millis();
  settleMs = 400; // Unknown initial shaft position.
#if defined(ARDUINO_UNOWIFIR4) && !ROBOT_ENABLE_STANDALONE
  radarPwmOk = radarPwm.begin(50.0f, centrePulse / 200.0f);
#elif !defined(ARDUINO) && !ROBOT_ENABLE_STANDALONE
  radarPwmOk = true; // Host fixture has no hardware timer.
#endif
}

void radarAim(int degrees) {
  if (!radarReady()) return;
  degrees = constrain(degrees, -RADAR_MAX_DEGREES, RADAR_MAX_DEGREES);
  if (degrees == commandedBearing) return;
  const uint32_t now = millis();
  const bool stillMoving = (uint32_t)(now - aimChangedMs) < settleMs;
  const int travel = stillMoving ? 2 * RADAR_MAX_DEGREES : abs(degrees - commandedBearing);
  settleMs = 60 + 3 * travel; // Includes a 20 ms PWM frame and provisional mechanical settling.
  commandedBearing = degrees;
  aimChangedMs = now;
  ++aimGeneration;
#if defined(ARDUINO_UNOWIFIR4)
  radarPwmOk = radarPwm.pulseWidth_us(centrePulse + abs(degrees) * ((degrees < 0 ? rightPulse : leftPulse) - centrePulse) / 60);
#endif
}

static bool radarSettled(uint32_t now) {
  return radarReady() && (uint32_t)(now - aimChangedMs) >= settleMs;
}

void captureEcho(uint8_t sensor) {
  if (!echoArmed || sensor != activeSensor) return;
  uint32_t now = micros();
  if (digitalRead(echoPins[sensor]) == HIGH) {
    echoStartUs = now; echoRisen = true;
  } else if (echoRisen) {
    echoWidthUs = (uint32_t)(now - echoStartUs);
    echoReady = true; echoArmed = false;
  }
}
void echoChanged() { captureEcho(FRONT_SENSOR); }
void rightEchoChanged() { captureEcho(RIGHT_SENSOR); }
void leftEchoChanged() { captureEcho(2); }

void acceptReading(uint8_t sensor, int cm, uint32_t width) {
  if (sensor == FRONT_SENSOR && radarEnabled) {
    if (!radarSettled(millis()) || (pingPending && pingGeneration != aimGeneration)) return;
    sampleGeneration = aimGeneration;
  }
  samplePoses[sensor]=pingPending?pingPose:odometry.pose;
  ++samples[sensor].sequence;
  samples[sensor].sampledMs = pingPending?pingStartMs:millis();
  samples[sensor].valid = cm >= 2 && cm <= 400;
  // A completed long pulse is distinct from a missing or stuck signal.
  samples[sensor].status = samples[sensor].valid ? RANGE_VALID :
      (width > 23200 && width <= 40000 ? RANGE_NO_RETURN : RANGE_INVALID);
  samples[sensor].cm = samples[sensor].valid ? cm : -1;
  samples[sensor].echoUs = width;
}

void setupSensor() {
  for (uint8_t sensor = 0; sensor < sonarCount; ++sensor) {
    digitalWrite(triggerPins[sensor], LOW); pinMode(triggerPins[sensor], OUTPUT);
    pinMode(echoPins[sensor], INPUT);
  }
  pinMode(GREEN_LED, INPUT); // D11 reserved for left ECHO, even before opt-in.
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoChanged, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ECHO_PIN), rightEchoChanged, CHANGE);
  lastPingMs = millis() - PING_MS;
}

void serviceSensor() {
  uint32_t now = millis();
  bool ready; uint32_t width;
  noInterrupts();
  ready = echoReady; width = echoWidthUs; echoReady = false;
  interrupts();
  if (pingPending && ready) {
    acceptReading(activeSensor, width >= 116 && width <= 23200 ? (int)(width / 58) : -1, width);
    pingPending=false;
  } else if (pingPending && (uint32_t)(micros() - pingStartUs) >= ECHO_TIMEOUT_US) {
    noInterrupts();
    echoArmed = false; echoReady = false;
    interrupts();
    acceptReading(activeSensor, -1, 0); pingPending = false;
  }
  const uint32_t interval = radarEnabled && pingInterval < 60 ? 60 : pingInterval;
  if (!pingPending && (uint32_t)(now - lastPingMs) >= interval) {
    // In the arena the scanner owns the next settled slot. Side units can
    // still ping during a slew. Wall modes keep their normal round robin.
    if ((mode==RADAR_SCAN || mode==OBJECT) && radarSettled(now))
      nextSensor=FRONT_SENSOR;
    // Let a fixed sonar use the slot while the scanner is slewing.
    if (nextSensor == FRONT_SENSOR && radarEnabled && !radarSettled(now)) nextSensor = RIGHT_SENSOR;
    lastPingMs = now;
    noInterrupts(); echoArmed = false; activeSensor = nextSensor; interrupts();
    nextSensor = (nextSensor+1)%sonarCount;
    pingGeneration = aimGeneration;
    // A failed sensor still yields its next slot to the other sensor.
    if (digitalRead(echoPins[activeSensor]) == HIGH) { acceptReading(activeSensor, -1, 0); return; }
    noInterrupts(); echoRisen = false; echoReady = false; echoArmed = true; interrupts();
    pingStartMs=now;pingPose=odometry.pose;pingStartUs = micros(); pingPending = true;
    digitalWrite(triggerPins[activeSensor], HIGH); delayMicroseconds(10);
    digitalWrite(triggerPins[activeSensor], LOW);
  }
}

static SensorReading acquiredReading(uint8_t sensor) {
  if (sensor >= sonarCount) return {-1, 0, 0, RANGE_INVALID};
  const DistanceSample& sample = samples[sensor];
  RangeStatus status = (uint32_t)(millis() - sample.sampledMs) >= STALE_MS ? RANGE_STALE : sample.status;
  return {status == RANGE_VALID ? sample.echoUs / 58.0f : -1.0f, sample.sampledMs, sample.sequence, status};
}

SensorReading radarReading() {
  if (!radarReady() || sampleGeneration != aimGeneration) return {-1, 0, 0, RANGE_INVALID};
  return acquiredReading(FRONT_SENSOR);
}

SensorReading sensorReading(uint8_t sensor) {
  if (sensor == FRONT_SENSOR && radarEnabled) {
    if (commandedBearing != 0) return {-1, 0, 0, RANGE_INVALID};
    return radarReading();
  }
  return acquiredReading(sensor);
}

float sensorDistanceCm(uint8_t sensor) { return sensorReading(sensor).cm; }

bool leftSonarEnabled(){return sonarCount==3;}
maze::Pose sensorPose(uint8_t sensor){return sensor<sonarCount?samplePoses[sensor]:maze::Pose();}
bool configureSonars(bool left,uint32_t interval){
  if(interval<25||interval>100||(!left&&interval<35))return false;
  stopRobot();
  noInterrupts();echoArmed=false;echoReady=false;interrupts();pingPending=false;
  if(sonarCount==3)detachInterrupt(digitalPinToInterrupt(11));
  sonarCount=left?3:2;pingInterval=interval;nextSensor=0;
  for(auto &s:samples)s=DistanceSample();

  if(left){digitalWrite(0,LOW);pinMode(0,OUTPUT);pinMode(11,INPUT);attachInterrupt(digitalPinToInterrupt(11),leftEchoChanged,CHANGE);}
  else pinMode(11,INPUT);
  if (!radarEnabled) lastPingMs=millis()-interval;
  return true;
}

uint32_t sonarInterval(){return pingInterval;}
