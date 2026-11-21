#pragma once
#include "WallNavigator.h"
#ifndef RADAR_FACE_AHEAD_OF_PIVOT_MM
#define RADAR_FACE_AHEAD_OF_PIVOT_MM 0
#endif

// Include after the SensorReading declaration. radarReading() must deliver
// only samples triggered at the current settled command, including misses.
void radarAim(int degrees);
void radarPoint(uint16_t sweepId,int degrees,int mm,int segment);
void radarLock(int degrees,int rangeMm,int widthMm);

namespace arena {
class Search {
 public:
  enum State { IDLE,SCAN,PIVOT,DRIVE,DONE,EDGE_BRAKE,EDGE_RETRACE,FAULT } state=IDLE;
  const char* reason="OBJECT_IDLE";
  void setProfile(float low,float high){minWidth=low;maxWidth=high;}
  void setEntryClearance(float mm){entryRadius=isfinite(mm)&&mm>0?mm:0;}
  void stop(){state=IDLE;reason="OBJECT_IDLE";radarAim(0);}
  void begin(const wall::Input& i){
    trailHead=trailCount=edgeHits=0;edgeKnown=turningInward=false;recordTrail(i.pose,true);
    entry=i.pose;surveyHeading=i.pose.heading;framed=false;obstacleRange=INFINITY;started=i.now;headings=1;views=retries=0;locked=false;
    for(auto& s:history)s.count=0;
    slot=0;hasTraversed=followRoute=false;
    scan(i,DISCOVER,-60,60,5);
  }
  wall::Speeds step(const wall::Config& c,const wall::Input& i,uint8_t tape,const SensorReading& r){
    if(state==IDLE||state==FAULT||state==DONE)return {};
#ifdef ARDUINO
    planningStartedUs=micros();
#endif
    if(!c.valid()||RADAR_FACE_AHEAD_OF_PIVOT_MM<0||RADAR_FACE_AHEAD_OF_PIVOT_MM>c.frontSensor){fail("OBJECT_CALIBRATION_REQUIRED");return {};}
    if(i.now-started>150000){fail("OBJECT_SEARCH_TIMEOUT");return {};}
    if(state==EDGE_BRAKE||state==EDGE_RETRACE)return recoverEdge(c,i,tape);
    if(tape){
      if(!turningInward){edgeKnown=true;edgeContact=i.pose;}edgeMask=tape;edgeAt=quietAt=i.now;
      edgeQuietPose=i.pose;locked=false;followRoute=false;
      state=EDGE_BRAKE;reason="OBJECT_EDGE_BRAKING";radarAim(0);
      if(++edgeHits>5)fail("OBJECT_REPEATED_EDGE_CONTACT");
      return {};
    }
    recordTrail(i.pose,false);
    if(locked&&i.now-lastLock>45000){fail("OBJECT_TARGET_ESTIMATE_EXPIRED");return {};}
    if(state==SCAN){acquire(c,i,r);return {};}
    if(state==DRIVE)return drive(c,i,r);
    if(i.now-motionAt>8000||wall::displacement(motionPose,i.pose)>c.distanceTolerance){
      fail("OBJECT_PIVOT_DRIFT_OR_TIMEOUT");return {};
    }
    if(i.now-evidenceAt>kLifetime){fail("OBJECT_CLEARANCE_EXPIRED");return {};}
    if(r.status==RANGE_VALID&&isfinite(r.cm)&&i.now-r.sampledMs<350
        &&r.cm*10+c.frontSensor<=c.radius()+c.margin){fail("OBJECT_PIVOT_BLOCKED");return {};}
    wall::Config turnConfig=c;turnConfig.angleTolerance=fmaxf(.035f,c.angleTolerance*.5f);
    auto speeds=turn.step(turnConfig,i);
    if(turn.failed){fail("OBJECT_PIVOT_DRIFT");return {};}
    if(turn.done){
      if(turningInward){turningInward=false;edgeHits=0;}
      if(afterTurn==CORRIDOR||afterTurn==VIEW)scan(i,afterTurn,-60,60,5);
      else if(afterTurn==VERIFY)trackScan(c,i);
      else scan(i,DISCOVER,-60,60,5);
      return {};
    }
    return speeds;
  }

 private:
#ifdef ARDUINO
  uint32_t planningStartedUs=0;
  bool timeAvailable()const{return (uint32_t)(micros()-planningStartedUs)<100000;}
#else
  bool timeAvailable()const{return true;}
#endif
  static constexpr float rad=maze::PI_F/180;
  static constexpr uint32_t kLifetime=22000;
  enum Purpose {DISCOVER,VERIFY,CORRIDOR,VIEW,RESCAN};
  struct Sweep {
    maze::Pose pose; uint32_t stamp=0;
    float cs=1,sn=0;int16_t mm[61]; int first=0,spacing=5,count=0;
  } history[5];
  struct Candidate {float bearing=0,range=0,width=0,span=0;};
  int slot=0,lastAngle=0,segment=0,headings=0,views=0,retries=0;
  uint16_t sweepId=0;
  uint32_t started=0,aimed=0,sequence=0,motionAt=0,evidenceAt=0,lastLock=0,traversedAt=0;
  maze::Pose entry,motionPose,traversedStart;
  float traversedLength=0;bool hasTraversed=false,followRoute=false;
  bool framed=false;float surveyHeading=0,obstacleBearing=0,obstacleRange=INFINITY;
  float minWidth=0,maxWidth=0,entryRadius=0,targetX=0,targetY=0,lastWidth=0,travel=0,viewLength=0,viewFocusX=0,viewFocusY=0;
  bool locked=false,detour=false,settled=false;
  Purpose purpose=DISCOVER,afterTurn=DISCOVER;
  wall::Turn turn;
  void fail(const char* why){state=FAULT;reason=why;radarAim(0);}
  // Wheel coordinates retain both straight travel and the sense of a pivot.
  // Rewind these points in reverse order so a grazing contact never sends
  // the rear of the car blindly sideways. One entry every ~15 mm of wheel travel.
  struct WheelPoint {float left,right;};
  static constexpr int trailSize=48;
  WheelPoint trail[trailSize];
  int trailHead=0,trailCount=0,edgeHits=0;
  bool edgeKnown=false,turningInward=false;
  uint8_t edgeMask=0;
  uint32_t edgeAt=0,quietAt=0;
  maze::Pose edgeContact,edgeQuietPose;
  void recordTrail(const maze::Pose& p,bool force){
    if(trailCount&&!force){
      const auto& prev=trail[(trailHead+trailSize-1)%trailSize];
      if(fmaxf(fabsf(p.left-prev.left),fabsf(p.right-prev.right))<15)return;
    }
    trail[trailHead]={p.left,p.right};trailHead=(trailHead+1)%trailSize;
    if(trailCount<trailSize)++trailCount;
  }
  wall::Speeds recoverEdge(const wall::Config& c,const wall::Input& i,uint8_t tape){
    if(i.now-edgeAt>16000){fail("OBJECT_EDGE_RECOVERY_TIMEOUT");return {};}
    edgeMask|=tape;
    if(state==EDGE_BRAKE){
      if(i.pose.left!=edgeQuietPose.left||i.pose.right!=edgeQuietPose.right){
        quietAt=i.now;edgeQuietPose=i.pose;
      }
      if(i.now-quietAt<200 || fabsf(i.leftSpeed)>=1 || fabsf(i.rightSpeed)>=1)return {};
      state=EDGE_RETRACE;reason="OBJECT_EDGE_RETRACING";
    }
    // First move back onto known floor. Test clearance only at recorded
    // waypoints, while stationary, before the chassis corners swing around.
    float retreat=wall::displacement(edgeContact,i.pose);
    while(trailCount){
      int index=(trailHead+trailSize-1)%trailSize;
      float l=trail[index].left-i.pose.left,r=trail[index].right-i.pose.right;
      float remaining=fmaxf(fabsf(l),fabsf(r));
      if(remaining>c.distanceTolerance){
        float speed=fminf(fminf(60,c.cruise),sqrtf(2*c.deceleration*remaining));
        return {fabsf(l)>c.distanceTolerance?speed*l/remaining:0,
                fabsf(r)>c.distanceTolerance?speed*r/remaining:0};
      }
      if(!tape && (retreat>=c.radius()+c.margin || trailCount==1)){
        if(fabsf(i.leftSpeed)>=1 || fabsf(i.rightSpeed)>=1)return {};
        if(footprint(c,i,0,0,true,evidenceAt)){
          float inward=retreat>c.distanceTolerance?
              atan2f(i.pose.y-edgeContact.y,i.pose.x-edgeContact.x):edgeContact.heading+maze::PI_F;
          float delta=maze::wrap(inward-i.pose.heading);
          // On a head-on approach either half-turn faces back inside.
          // Prefer turning away from the outer sensor that touched first.
          if(fabsf(fabsf(delta)-maze::PI_F)<.12f)
            delta=(edgeMask&4)&&!(edgeMask&1)?-fabsf(delta):fabsf(delta);
          surveyHeading=i.pose.heading+delta;headings=1;views=retries=0;
          framed=detour=hasTraversed=false;obstacleRange=INFINITY;
          motionAt=i.now;motionPose=i.pose;afterTurn=DISCOVER;
          turn.begin(delta,i.pose);turningInward=true;state=PIVOT;reason="OBJECT_TURNING_INWARD";
          return {};
        }
      }
      trailHead=index;--trailCount;
    }
    fail("OBJECT_EDGE_NO_CHECKED_TURN");return {};
  }
  void scan(const wall::Input& i,Purpose p,int first,int last,int spacing){
    slot=(slot+1)%5;
    Sweep& s=history[slot];s.count=0;s.first=first;s.spacing=spacing;s.pose=i.pose;s.stamp=i.now;
    lastAngle=last;segment=0;purpose=p;settled=false;aimed=i.now;++sweepId;
    state=SCAN;reason=p==DISCOVER?"OBJECT_DISCOVERY_SWEEP":p==VERIFY?"OBJECT_CONFIRMING_CONTOUR":
      p==VIEW?"OBJECT_CHECKING_VIEWPOINT":p==RESCAN?"OBJECT_REACQUIRING":"OBJECT_CHECKING_CORRIDOR";
    radarAim(first);
  }
 public:
  static bool adjacent(int a,int b,int spacing){
    if(a<=0||b<=0)return false;
    const float near=fminf(a,b),jump=fabsf(float(a-b));
    return jump<=35+.06f*near && hypotf(jump,near*spacing*rad)<60+.16f*near;
  }
 private:
  void acquire(const wall::Config& c,const wall::Input& i,const SensorReading& r){
    Sweep& s=history[slot];
    if(i.now-s.stamp>15000){fail("OBJECT_SWEEP_TIMEOUT");return;}
    if(fmaxf(fabsf(i.leftSpeed),fabsf(i.rightSpeed))>=1){
      if(settled){fail("OBJECT_MOVED_DURING_SCAN");return;}
      aimed=i.now;return;
    }
    if(!settled){
      if(i.now-aimed<160)return;
      settled=true;s.pose=i.pose;s.cs=cosf(i.pose.heading);s.sn=sinf(i.pose.heading);s.stamp=i.now;sequence=r.sequence;aimed=i.now;return;
    }
    if(wall::displacement(s.pose,i.pose)>2||maze::angleDistance(s.pose.heading,i.pose.heading)>rad){
      fail("OBJECT_MOVED_DURING_SCAN");return;
    }
    if(i.now-aimed>1800){fail("OBJECT_RADAR_TIMEOUT");return;}
    if(r.sequence==sequence||(int32_t)(r.sampledMs-aimed)<=0||i.now-r.sampledMs>350||r.status==RANGE_STALE)return;
    sequence=r.sequence;
    int mm=r.status==RANGE_VALID&&isfinite(r.cm)&&r.cm>=2&&r.cm<=400?(int)lroundf(10*r.cm):-1;
    int n=s.count,angle=s.first+n*s.spacing;
    s.mm[n]=mm;
    if(mm>0&&(n==0||!adjacent(s.mm[n-1],mm,s.spacing)))++segment;
    radarPoint(sweepId,angle,mm,mm>0?segment:0);
    ++s.count;
    if(angle+s.spacing<=lastAngle){aimed=i.now;radarAim(angle+s.spacing);return;}
    if(purpose==CORRIDOR){planDrive(c,i);return;}
    if(purpose==VIEW){chooseView(c,i);return;}
    identify(c,i);
  }
  void relative(const wall::Config& c,const wall::Input& i,float& angle,float& range)const{
    float h=i.pose.heading,pivot=c.frontSensor-RADAR_FACE_AHEAD_OF_PIVOT_MM;
    float dx=targetX-i.pose.x-pivot*cosf(h),dy=targetY-i.pose.y-pivot*sinf(h);
    angle=maze::wrap(atan2f(dy,dx)-h)/rad;range=hypotf(dx,dy)-RADAR_FACE_AHEAD_OF_PIVOT_MM;
  }
  void remember(const wall::Config& c,const wall::Input& i,const Candidate& a){
    float h=i.pose.heading,b=h+a.bearing*rad;
    float pivot=c.frontSensor-RADAR_FACE_AHEAD_OF_PIVOT_MM,r=a.range+RADAR_FACE_AHEAD_OF_PIVOT_MM;
    targetX=i.pose.x+pivot*cosf(h)+r*cosf(b);
    targetY=i.pose.y+pivot*sinf(h)+r*sinf(b);lastWidth=a.width;
  }
  void trackScan(const wall::Config& c,const wall::Input& i){
    float angle,range;relative(c,i,angle,range);
    int half=(int)ceilf(atan2f(lastWidth*.6f,fmaxf(20,range))/rad)+8;
    int first=(int)fmaxf(-60,floorf(angle)-half),last=(int)fminf(60,ceilf(angle)+half);
    if(last==60&&(last-first)%2&&first>-60)--first;
    if(last-first<4){reacquire(i);return;}
    scan(i,VERIFY,first,last,2);
  }
  void reacquire(const wall::Input& i){
    if(++retries>2){fail("OBJECT_REACQUISITION_EXHAUSTED");return;}
    scan(i,locked?RESCAN:VIEW,-60,60,5);
  }
  void identify(const wall::Config& c,const wall::Input& i){
    if(!isfinite(minWidth)||!isfinite(maxWidth)||minWidth<=0||maxWidth<minWidth){
      fail("OBJECT_TARGET_PROFILE_REQUIRED");return;
    }
    const Sweep& s=history[slot];
    float predicted=0,distance=0;relative(c,i,predicted,distance);
    Candidate chosen;int matches=0;bool occluded=false;float clipped=0;
    for(int n=0;n<s.count;){
      if(s.mm[n]<=0){++n;continue;}
      int first=n++,sum=s.mm[first];
      while(n<s.count&&adjacent(s.mm[n-1],s.mm[n],s.spacing))sum+=s.mm[n++];
      int count=n-first;
      float a=s.first+first*s.spacing,b=s.first+(n-1)*s.spacing;
      float range=float(sum)/count;
      if((locked||purpose==VERIFY)&&predicted>=a-s.spacing&&predicted<=b+s.spacing
          && range<distance-fmaxf(30,.08f*distance))occluded=true;
      if(first==0||n==s.count||count<2){
        if(count>=2&&count<s.count/2&&(first==0||n==s.count))clipped=(a+b)*.5f;
        continue;
      } // Both contour edges must be visible.
      Candidate x;x.range=range;x.bearing=(a+b)*.5f;x.span=b-a+s.spacing;
      x.width=2*range*sinf(x.span*rad*.5f);
      float allowance=purpose!=VERIFY?range*s.spacing*rad:0;
      if(x.width+allowance<minWidth||x.width-allowance>maxWidth){
        if(purpose==DISCOVER&&!locked&&range<obstacleRange){
          obstacleRange=range;obstacleBearing=i.pose.heading+x.bearing*rad;
        }
        continue;
      }
      if((locked||purpose==VERIFY)&&(fabsf(x.bearing-predicted)>8||
          fabsf(range-distance)>fmaxf(35,.10f*distance)))continue;
      if(locked&&fabsf(lastWidth-x.width)>fmaxf(18,.25f*lastWidth)+allowance)continue;
      ++matches;chosen=x;
    }
    if(matches==1&&!occluded){
      if(purpose==DISCOVER||purpose==RESCAN){
        if(!locked)remember(c,i,chosen);
        trackScan(c,i);return;
      }
      remember(c,i,chosen);locked=true;lastLock=i.now;
      radarLock((int)lroundf(chosen.bearing),(int)lroundf(chosen.range),(int)lroundf(chosen.width));
      float centreBearing=maze::wrap(atan2f(targetY-i.pose.y,targetX-i.pose.x)-i.pose.heading)/rad;
      if(fabsf(chosen.bearing)>(hasTraversed?7:fmaxf(3,.5f*c.angleTolerance/rad))){
        pivot(c,i,centreBearing,VERIFY);return;
      }
      detour=false;
      // Fine scans with a measured forward return can use earlier fresh
      // clearance at this pose. Otherwise collect a dedicated corridor scan.
      int centre=(int)lroundf(-float(s.first)/s.spacing);
      bool surveyed=false;
      for(const Sweep& prior:history){
        if(prior.first==-60&&prior.spacing==5&&prior.count==25&&i.now-prior.stamp<8000
           &&wall::displacement(prior.pose,i.pose)<2&&maze::angleDistance(prior.pose.heading,i.pose.heading)<rad)surveyed=true;
      }
      if(surveyed&&centre>=0&&centre<s.count&&abs(s.first+centre*s.spacing)<=1&&s.mm[centre]>0)
        planDrive(c,i,true);
      else scan(i,CORRIDOR,-60,60,5);
      return;
    }
    if(purpose==RESCAN){chooseView(c,i);return;}
    if(purpose==VERIFY&&clipped&&fabsf(predicted)>35){
      float bearing=maze::wrap(atan2f(targetY-i.pose.y,targetX-i.pose.x)-i.pose.heading);
      pivot(c,i,bearing/rad,VERIFY);return;
    }
    if(locked||purpose==VERIFY){reacquire(i);return;}
    if(matches>1){scan(i,VIEW,-60,60,5);reason="OBJECT_AMBIGUOUS_CONTOURS";return;}
    if(clipped&&!framed){
      framed=true;pivot(c,i,clipped*.8f,DISCOVER);return;
    }
    if(headings<4){
      float next=surveyHeading+headings*90*rad;++headings;
      pivot(c,i,maze::wrap(next-i.pose.heading)/rad,DISCOVER);return;
    }
    if(isfinite(obstacleRange)&&maze::angleDistance(obstacleBearing,i.pose.heading)>12*rad){
      pivot(c,i,maze::wrap(obstacleBearing-i.pose.heading)/rad,VIEW);return;
    }
    scan(i,VIEW,-60,60,5);
  }
  // Cover each tile of the inflated swept footprint with a disk. A tile needs
  // either the measured entry disk, the current occupied body, or valid echoes
  // beyond it across its entire angular extent. Missing sectors remain unknown.
  struct Location {
    maze::Pose pose;
    uint32_t now;
    Location(const wall::Input& input):pose(input.pose),now(input.now){}
  };
  bool known(const wall::Config& c,const Location& i,float wx,float wy,float pad,uint32_t& oldest)const{
    if(hypotf(wx-entry.x,wy-entry.y)+pad<=entryRadius)return true;
    float dx=wx-i.pose.x,dy=wy-i.pose.y,ch=cosf(i.pose.heading),sh=sinf(i.pose.heading);
    float bx=dx*ch+dy*sh,by=dy*ch-dx*sh;
    if(bx-pad>=-c.rear&&bx+pad<=c.front&&fabsf(by)+pad<=c.halfWidth)return true;
    for(int k=0;k<5;++k){
      const Sweep& s=history[(slot+5-k)%5];uint32_t age=i.now-s.stamp;
      if(s.count<2||age>=kLifetime)continue;
      float h=s.pose.heading,cs=s.cs,sn=s.sn;
      float pivot=c.frontSensor-RADAR_FACE_AHEAD_OF_PIVOT_MM;
      float x=wx-s.pose.x-pivot*cs,y=wy-s.pose.y-pivot*sn,d=hypotf(x,y);
      if(d<=pad+RADAR_FACE_AHEAD_OF_PIVOT_MM)continue;
      float a=maze::wrap(atan2f(y,x)-h)/rad,w=asinf(pad/d)/rad;
      int lo=(int)floorf((a-w-s.first)/s.spacing),hi=(int)ceilf((a+w-s.first)/s.spacing);
      if(lo<0||hi>=s.count)continue;
      bool clear=true;
      for(int j=lo;j<=hi;++j)if(s.mm[j]<=0||s.mm[j]+RADAR_FACE_AHEAD_OF_PIVOT_MM<=d+pad){clear=false;break;}
      if(clear){if(age>i.now-oldest)oldest=s.stamp;return true;}
    }
    return false;
  }
  // The preceding checked straight segment establishes clearance beside the
  // wheels while the scanner looks ahead. Reuse only its actually traversed
  // length, while recent, and require the entire tile inside its rectangle.
  bool inTraversed(const wall::Config& c,const Location& i,float wx,float wy,
                   float heading,float sx,float sy)const{
    if(!hasTraversed||i.now-traversedAt>=kLifetime)return false;
    float h=traversedStart.heading,co=cosf(h),si=sinf(h),dx=wx-traversedStart.x,dy=wy-traversedStart.y;
    float x=dx*co+dy*si,y=dy*co-dx*si,a=heading-h;
    float ex=.5f*(sx*fabsf(cosf(a))+sy*fabsf(sinf(a)));
    float ey=.5f*(sy*fabsf(cosf(a))+sx*fabsf(sinf(a)));
    float m=c.margin+2*c.distanceTolerance;
    return x-ex>=-c.rear-m-.01f && x+ex<=traversedLength+c.front+m+.01f && fabsf(y)+ey<=c.halfWidth+m+.01f;
  }
  bool footprint(const wall::Config& c,const Location& i,float bearing,float length,bool turning,uint32_t& oldest)const{
    float h=i.pose.heading+bearing*rad,ch=cosf(h),sh=sinf(h);
    float margin=c.margin+2*c.distanceTolerance;
    float radius=c.radius()+margin,side=turning?radius:c.halfWidth+margin;
    float front=turning?radius:c.front+margin+length,rear=turning?radius:c.rear+margin;
    // Even a known entry disk may now contain a measured obstruction.
    for(const auto& s:history){
      if(!timeAvailable())return false;
      if(i.now-s.stamp>=kLifetime)continue;
      for(int j=0;j<s.count;++j){
        if(s.mm[j]<=0)continue;
        float a=s.pose.heading+(s.first+j*s.spacing)*rad;
        float pivot=c.frontSensor-RADAR_FACE_AHEAD_OF_PIVOT_MM,r=s.mm[j]+RADAR_FACE_AHEAD_OF_PIVOT_MM;
        float dx=s.pose.x+pivot*s.cs+r*cosf(a)-i.pose.x;
        float dy=s.pose.y+pivot*s.sn+r*sinf(a)-i.pose.y;
        float x=dx*ch+dy*sh,y=dy*ch-dx*sh;
        if(turning?hypotf(x,y)<=radius:(x>=-rear&&x<=front&&fabsf(y)<=side))return false;
      }
    }
    const int nx=(int)ceilf((front+rear)/30),ny=(int)ceilf(2*side/30);
    if(nx<1||ny<1||nx>100||ny>100)return false;
    float sx=(front+rear)/nx,sy=2*side/ny,pad=.5f*hypotf(sx,sy);
    oldest=i.now;
    if(turning&&wall::displacement(entry,i.pose)+radius<=entryRadius)return true;
    for(int j=0;j<nx;++j)for(int k=0;k<ny;++k){
      if(!timeAvailable())return false;
      float x=-rear+(j+.5f)*sx,y=-side+(k+.5f)*sy;
      if(turning&&hypotf(x,y)>radius+pad)continue;
      float wx=i.pose.x+x*ch-y*sh,wy=i.pose.y+x*sh+y*ch;
      if(inTraversed(c,i,wx,wy,h,sx,sy)){
        if(i.now-traversedAt>i.now-oldest)oldest=traversedAt;
        continue;
      }
      if(known(c,i,wx,wy,pad,oldest))continue;
      // At a boundary the enclosing disk can straddle two valid source
      // regions. Refine only these tiles instead of densifying the whole map.
      for(int m=-1;m<=1;m+=2)for(int n=-1;n<=1;n+=2){
        float qx=wx+(m*sx*ch-n*sy*sh)*.25f,qy=wy+(m*sx*sh+n*sy*ch)*.25f;
        if(inTraversed(c,i,qx,qy,h,sx*.5f,sy*.5f)){
          if(i.now-traversedAt>i.now-oldest)oldest=traversedAt;
        }else if(!known(c,i,qx,qy,pad*.5f,oldest))return false;
      }
    }
    return true;
  }
  void pivot(const wall::Config& c,const wall::Input& i,float degrees,Purpose next){
    if(!footprint(c,i,0,0,true,evidenceAt)){fail("OBJECT_PIVOT_CLEARANCE_REQUIRED");return;}
    motionAt=i.now;motionPose=i.pose;followRoute=false;afterTurn=next;
    turn.begin(degrees*rad,i.pose);state=PIVOT;reason="OBJECT_CHECKED_PIVOT";radarAim(0);
  }
  void chooseView(const wall::Config& c,const wall::Input& i){
    if(views>=2){fail("OBJECT_VIEWPOINTS_EXHAUSTED");return;}
    uint32_t stamp;
    // Require actual positive returns on the proposed side as well as a
    // checked swept footprint; entry commissioning alone cannot select a side.
    const Sweep& s=history[slot];
    int best=0;float score=0;
    for(int direction=-1;direction<=1;direction+=2){
      for(int offset=50;offset>=40;offset-=10){
        int angle=direction*offset,n=(angle-s.first)/s.spacing;
        int reach=s.mm[n];
        if(s.mm[n-1]<=0||s.mm[n+1]<=0)continue;
        for(int length=180;length>=100;length-=40){
          float gain=length*sinf(offset*rad);
          if(gain<=score||reach<=c.front-c.frontSensor+c.margin+length)continue;
          if(footprint(c,i,angle,length,false,stamp)){
            best=angle;score=gain;viewLength=length;break;
          }
        }
      }
    }
    if(!best){fail(timeAvailable()?"OBJECT_NO_CHECKED_VIEWPOINT":"OBJECT_PLANNER_TIME_LIMIT");return;}
    float toward=locked?0:(isfinite(obstacleRange)?maze::wrap(obstacleBearing-i.pose.heading):0);
    float look=isfinite(obstacleRange)?obstacleRange+c.frontSensor:700;
    viewFocusX=locked?targetX:i.pose.x+look*cosf(i.pose.heading+toward);
    viewFocusY=locked?targetY:i.pose.y+look*sinf(i.pose.heading+toward);
    ++views;detour=true;pivot(c,i,best,CORRIDOR);
  }
  void planDrive(const wall::Config& c,const wall::Input& i,bool preview=false){
    const Sweep& s=history[slot];
    int centre=(int)lroundf(-float(s.first)/s.spacing);
    int front=s.mm[centre];
    if(front<=0){if(detour)fail("OBJECT_VIEWPOINT_RANGE_UNKNOWN");else reacquire(i);return;}
    float room=viewLength;
    if(!detour){
      float bearing,distance;relative(c,i,bearing,distance);
      float tolerance=fmaxf(35,.10f*distance);
      if(!locked||i.now-lastLock>kLifetime||fabsf(bearing)>8||fabsf(front-distance)>tolerance){
        if(preview)scan(i,CORRIDOR,-60,60,5);else reacquire(i);return;
      }
      room=fminf(front,distance)+c.frontSensor-c.front-200;
      // A stopped estimate can differ by one encoder edge from the
      // braking threshold, plus rounding of the stored mm return.
      float arrivalTolerance=c.distanceTolerance+c.distanceTolerance/1.5f+.5f;
      if(room<=arrivalTolerance){
        if(room < -arrivalTolerance){fail("OBJECT_STANDOFF_OVERRUN");return;}
        state=DONE;reason="OBJECT_REACHED";radarAim(0);return;
      }
    }
    travel=detour?room:fminf(120,room);
    Location path(i);
    if(followRoute){
      // Hold the last approved straight centreline across encoder ticks;
      // the checked envelope includes the bounded lateral/heading error.
      float h=motionPose.heading,dx=i.pose.x-motionPose.x,dy=i.pose.y-motionPose.y;
      float along=dx*cosf(h)+dy*sinf(h);
      path.pose.heading=h;path.pose.x=motionPose.x+along*cosf(h);path.pose.y=motionPose.y+along*sinf(h);
    }
    if(edgeKnown){
      float dx=edgeContact.x-path.pose.x,dy=edgeContact.y-path.pose.y;
      float h=path.pose.heading,along=dx*cosf(h)+dy*sinf(h),side=-dx*sinf(h)+dy*cosf(h);
      if(along>0 && along<=travel+c.margin && fabsf(side)<=c.halfWidth){
        if(detour)fail("OBJECT_VIEWPOINT_AT_EDGE");else scan(i,VIEW,-60,60,5);
        return;
      }
    }
    if(!footprint(c,path,0,travel,false,evidenceAt)){
      if(preview){scan(i,CORRIDOR,-60,60,5);return;}
      if(detour){fail("OBJECT_VIEWPOINT_BLOCKED");return;}
      chooseView(c,i);return;
    }
    state=DRIVE;reason=detour?"OBJECT_MOVING_VIEWPOINT":"OBJECT_APPROACHING";
    motionAt=i.now;motionPose=path.pose;followRoute=true;radarAim(0);
  }
  wall::Speeds drive(const wall::Config& c,const wall::Input& i,const SensorReading& r){
    if(i.now-motionAt>6500||i.now-evidenceAt>kLifetime){fail("OBJECT_MOTION_EVIDENCE_EXPIRED");return {};}
    float h=motionPose.heading,dx=i.pose.x-motionPose.x,dy=i.pose.y-motionPose.y;
    float along=dx*cosf(h)+dy*sinf(h);
    float lateral=dy*cosf(h)-dx*sinf(h),yaw=maze::wrap(h-i.pose.heading);
    float error=fabsf(lateral)+c.radius()*fabsf(yaw);
    if(error>2*c.distanceTolerance||along< -c.distanceTolerance){fail("OBJECT_DRIVE_DRIFT");return {};}
    float left=travel-along;
    if(left<=c.distanceTolerance){
      retries=0;traversedStart=motionPose;traversedLength=fminf(travel,along);traversedAt=i.now;hasTraversed=true;
      if(detour||!locked){
        headings=1;framed=false;obstacleRange=INFINITY;
        surveyHeading=atan2f(viewFocusY-i.pose.y,viewFocusX-i.pose.x);
        float aim=maze::wrap(surveyHeading-i.pose.heading)/rad;
        if(fabsf(aim)>30)pivot(c,i,aim,DISCOVER);
        else scan(i,DISCOVER,-60,60,5);
      }
      else trackScan(c,i);
      return {};
    }
    if(!detour&&i.now-lastLock>kLifetime){reacquire(i);return {};}
    if((int32_t)(r.sampledMs-motionAt)<=0||r.status==RANGE_STALE)return {};
    if(i.now-r.sampledMs>350||r.status!=RANGE_VALID||!isfinite(r.cm)||r.cm<2){
      if(detour)fail("OBJECT_VIEWPOINT_RANGE_UNKNOWN");else reacquire(i);
      return {};
    }
    float speed=fmaxf(fabsf(i.leftSpeed),fabsf(i.rightSpeed));
    float age=(i.now-r.sampledMs)*.001f,reading=r.cm*10;
    if(!detour){
      float bearing,distance;relative(c,i,bearing,distance);
      if(fabsf(bearing)>8||fabsf(reading-distance)>fmaxf(35,.10f*distance)+speed*age){reacquire(i);return {};}
    }
    float room=reading-speed*age+c.frontSensor-c.front-(detour?c.margin:200);
    if(room<=c.distanceTolerance){
      if(along>0){traversedStart=motionPose;traversedLength=fminf(travel,along);traversedAt=i.now;hasTraversed=true;}
      if(detour)fail("OBJECT_VIEWPOINT_BLOCKED");else trackScan(c,i);
      return {};
    }
    float remaining=fminf(room,left),reaction=c.deceleration*c.reaction;
    float cap=2*c.deceleration*remaining/(sqrtf(reaction*reaction+2*c.deceleration*remaining)+reaction);
    if(speed>cap+1)return {};
    float v=fminf(c.cruise,fminf(cap,remaining*2));
    float steering=wall::limit(yaw-lateral/fmaxf(120,c.front),-.15f,.15f);
    return {v*(1-steering),v*(1+steering)};
  }
};
}
