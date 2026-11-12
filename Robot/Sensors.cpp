#include "Robot.h"

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
  samplePoses[sensor]=pingPending?pingPose:odometry.pose;
  ++samples[sensor].sequence;
  samples[sensor].sampledMs = pingPending?pingStartMs:millis();
  samples[sensor].valid = cm >= 2 && cm <= 400;
  samples[sensor].status = samples[sensor].valid ? RANGE_VALID : RANGE_INVALID;
  samples[sensor].cm = samples[sensor].valid ? cm : -1;
  samples[sensor].echoUs = samples[sensor].valid ? width : 0;

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
    samplePoses[activeSensor]=pingPose;pingPending=false;
    // HC-SR04 reports no obstacle with a completed ~38 ms pulse. Wait for
    // its falling edge; a missing/stuck signal is not a no-obstacle reading.
    if (width > 23200 && width <= 40000) samples[activeSensor].status = RANGE_NO_RETURN;
  } else if (pingPending && (uint32_t)(micros() - pingStartUs) >= ECHO_TIMEOUT_US) {
    noInterrupts();
    echoArmed = false; echoReady = false;
    interrupts();
    pingPending = false; acceptReading(activeSensor, -1, 0);
  }
  for (uint8_t sensor = 0; sensor < sonarCount; ++sensor) {
    if (samples[sensor].valid && (uint32_t)(millis() - samples[sensor].sampledMs) >= STALE_MS)
      acceptReading(sensor, -1, 0);
  }
  if (!pingPending && (uint32_t)(now - lastPingMs) >= pingInterval) {
    lastPingMs = now;
    noInterrupts(); echoArmed = false; activeSensor = nextSensor; interrupts();
    nextSensor = (nextSensor+1)%sonarCount;
    // A failed sensor still yields its next slot to the other sensor.
    if (digitalRead(echoPins[activeSensor]) == HIGH) { acceptReading(activeSensor, -1, 0); return; }
    noInterrupts(); echoRisen = false; echoReady = false; echoArmed = true; interrupts();
    pingStartMs=now;pingPose=odometry.pose;pingStartUs = micros(); pingPending = true;
    digitalWrite(triggerPins[activeSensor], HIGH); delayMicroseconds(10);
    digitalWrite(triggerPins[activeSensor], LOW);
  }
}

float sensorDistanceCm(uint8_t sensor) {
  if (sensor >= sonarCount || !samples[sensor].valid ||
      (uint32_t)(millis() - samples[sensor].sampledMs) >= STALE_MS) return -1;
  return samples[sensor].echoUs / 58.0f;
}

// Expose acquisition time/identity so control never differentiates loop repeats.
SensorReading sensorReading(uint8_t sensor) {
  if (sensor >= sonarCount) return {-1, 0, 0, RANGE_INVALID};
  RangeStatus status = (uint32_t)(millis()-samples[sensor].sampledMs) >= STALE_MS ? RANGE_STALE : samples[sensor].status;
  return {sensorDistanceCm(sensor), samples[sensor].sampledMs, samples[sensor].sequence, status};
}

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
  lastPingMs=millis()-interval;return true;
}
