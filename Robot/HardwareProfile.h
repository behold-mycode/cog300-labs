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
 // Replace zero only with measurements from this car. No app configuration.
 p.leftMmPerEdge=0; p.rightMmPerEdge=0; // circumference / edges per revolution
 p.floorSensorOffset=0;             // axle to floor-sensor detection row
 p.leftSonarInstalled=false;         // enable only after third sonar is installed
 p.track=0;                           // wheel contact-centre spacing
 p.front=0; p.rear=0; p.halfWidth=0;   // full chassis envelope from axle centre
 p.frontSonar=0;                     // forward sonar origin from axle
 p.sideSonarX=0; p.sideSonarY=0;      // shared forward and lateral side offsets
 return p;
}
const Profile CAR=physicalCar();
}
