#pragma once
#include <math.h>
#include "CalibrationStore.h"
namespace hardware {
// Physical facts about this assembled car, NOT app settings. Units: mm.
// mm/edge = measured wheel travel / counted transitions (both edges counted).
struct Profile {
 bool leftSonarInstalled=false;
 float floorSensorOffset=0;
 bool invertA=true,invertB=false; // Previously established channel polarity: A right, B left.
 float leftMmPerEdge=0,rightMmPerEdge=0,track=0;
 float front=0,rear=0,halfWidth=0,frontSonar=0,sideSonarX=0,sideSonarY=0;
 bool valid()const{
  const float v[]={leftMmPerEdge,rightMmPerEdge,track,front,rear,halfWidth,frontSonar,sideSonarX,sideSonarY,floorSensorOffset};
  for(float x:v)if(!isfinite(x))return false;
  return floorSensorOffset>0&&floorSensorOffset<=front&&leftMmPerEdge>0&&rightMmPerEdge>0&&track>=10&&track<=1000&&front>0&&rear>0&&halfWidth>0&&
   frontSonar>=0&&frontSonar<=front&&sideSonarX>=-rear&&sideSonarX<=front&&sideSonarY>=0&&sideSonarY<=halfWidth&&
   fmaxf(leftMmPerEdge,rightMmPerEdge)*3/track<.25f;
 }
 uint32_t fingerprint()const{
  const float v[]={leftMmPerEdge,rightMmPerEdge,track,front,rear,halfWidth,frontSonar,sideSonarX,sideSonarY,floorSensorOffset};
  return persistent::crc(v,sizeof(v)) ^ (leftSonarInstalled?0x716bc125u:0) ^ (invertA?0xa135709du:0) ^ (invertB?0x69b24ec3u:0);
 }
};
// Fill only from actual measurements. The supplied module names do not encode
// these dimensions. Zero explicitly means unknown and prevents motion.
inline Profile physicalCar(){
 Profile p;
 // Reported 2026-11-17: approx. 65 mm wheels, 20 slots on each disc.
 // CHANGE interrupts count both slot edges: 40 edges per revolution.
 p.leftMmPerEdge=p.rightMmPerEdge=65.0f*3.14159265358979323846f/40.0f;
 p.floorSensorOffset=150;           // centre IR: 140 mm axle-to-body-front + 10 mm
 p.leftSonarInstalled=false;         // enable only after third sonar is installed
 p.track=130;                         // reported wheel contact-centre spacing
 p.front=150; p.rear=70; p.halfWidth=80; // 210 mm body; include 10 mm front protrusion
 p.frontSonar=150;                   // front sonar 10 mm ahead of body
 p.sideSonarX=45; p.sideSonarY=65;     // right sonar 15 mm inward from 80 mm half-width
 return p;
}
const Profile CAR=physicalCar();
}
