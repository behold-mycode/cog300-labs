#pragma once
#include <cassert>
#include "../Robot/Robot.h"
#include "../Robot/Motor.cpp"
#include "../Robot/Sensors.cpp"
#include "../Robot/Line.cpp"
#include "../Robot/Encoders.cpp"
#include "../Robot/MapTelemetry.cpp"
#include "../Robot/WallMode.cpp"
#include "../Robot/Navigation.cpp"
#include "../Robot/Protocol.cpp"
#include "../Robot/Persistence.cpp"
#include "../Robot/AutoSetup.cpp"
#include "../Robot/Standalone.cpp"
#include "../Robot/Wireless.cpp"
#include "../Robot/Robot.ino"
void command(const char* text) { char buffer[96];strcpy(buffer,text);processCommand(buffer); }
void tape(int mask) { levels[LINE_LEFT_PIN]=(mask>>2)&1;levels[LINE_CENTER_PIN]=(mask>>1)&1;levels[LINE_RIGHT_PIN]=mask&1; }
void tick(uint32_t ms,int front=60,int right=10) {
  nowMs+=ms;acceptReading(0,front,front>0?front*58U:0);acceptReading(1,right,right>0?right*58U:0);
  serviceOdometry();serviceObservation();serviceControlTimeout();serviceNavigation();serviceMotor();serviceMapExport();
}

hardware::Profile syntheticHardware(){hardware::Profile p;p.leftMmPerEdge=p.rightMmPerEdge=1;p.track=120;p.floorSensorOffset=90;p.leftSonarInstalled=true;p.front=90;p.rear=70;p.halfWidth=60;p.frontSonar=80;p.sideSonarX=30;p.sideSonarY=50;return p;}

// Tests for missing installation/calibration must not depend on the real car's
// compiled measurements being empty. Normal setup() still boots the real profile.
void setupUnconfigured(){
 setup();configureEncoders(0,false,false);configureSonars(false,70);
 configureHardware(hardware::Profile());
}
