#pragma once
#include <cassert>
#include "../Robot/Robot.h"
#include "../Robot/Motor.cpp"
#include "../Robot/Sensors.cpp"
#include "../Robot/Line.cpp"
#include "../Robot/Encoders.cpp"
#include "../Robot/MapTelemetry.cpp"
#include "../Robot/Navigation.cpp"
#include "../Robot/Protocol.cpp"
#include "../Robot/Robot.ino"
void command(const char* text) { char buffer[96];strcpy(buffer,text);processCommand(buffer); }
void tape(int mask) { levels[3]=(mask>>2)&1;levels[2]=(mask>>1)&1;levels[4]=mask&1; }
void tick(uint32_t ms,int front=60,int right=10) {
  nowMs+=ms;acceptReading(0,front,front>0?front*58U:0);acceptReading(1,right,right>0?right*58U:0);
  serviceOdometry();serviceObservation();serviceControlTimeout();serviceNavigation();serviceMotor();serviceMapExport();
}
