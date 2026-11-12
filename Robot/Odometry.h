#pragma once
#include <stdint.h>
#include <math.h>
#include "MazeGraph.h"
namespace maze {
struct Pose { float x=0,y=0,heading=0,path=0,left=0,right=0; };
class Odometry {
 public:
  Pose pose;
  bool configured=false, sampled=false;
  float leftScale=0,rightScale=0,track=0;
  uint32_t sampleMs=0;
  bool configure(float leftMm,float rightMm,float trackMm) {
    if(!isfinite(leftMm)||!isfinite(rightMm)||!isfinite(trackMm)||leftMm<=0||rightMm<=0||trackMm<=0) return false;
    leftScale=leftMm;rightScale=rightMm;track=trackMm;configured=true;sampled=false;return true;
  }
  void reset(int32_t l,int32_t r,uint32_t now) { pose=Pose();lastL=l;lastR=r;sampleMs=now;sampled=true; }
  void update(int32_t l,int32_t r,uint32_t now) {
    if(!configured) return;
    if(!sampled) { reset(l,r,now);return; }
    float dl=(int32_t)((uint32_t)l-(uint32_t)lastL)*leftScale;
    float dr=(int32_t)((uint32_t)r-(uint32_t)lastR)*rightScale;
    lastL=l;lastR=r;sampleMs=now;
    float dh=(dr-dl)/track, ds=(dl+dr)*0.5f;
    float scale=fabsf(dh)<0.00001f ? 1.0f : sinf(dh*0.5f)/(dh*0.5f);
    pose.x+=ds*scale*cosf(pose.heading+dh*0.5f);
    pose.y+=ds*scale*sinf(pose.heading+dh*0.5f);
    pose.heading+=dh;pose.path+=fabsf(ds);
    pose.left+=dl;pose.right+=dr;
  }
 private: int32_t lastL=0,lastR=0;
};
}
