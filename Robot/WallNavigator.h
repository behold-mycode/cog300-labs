#pragma once
#include "Odometry.h"
#include "Velocity.h"
namespace wall {
using maze::Pose;
struct Config {
 // Measured footprint extents from axle centre; sonar origins in body coordinates.
 float front=0,rear=0,halfWidth=0,frontSensor=0,sideX=0,sideY=0;
 float margin=0,deceleration=0,reaction=0,cruise=0,turnSpeed=0;
 float corridorWidth=0,track=0,distanceTolerance=0,angleTolerance=0,matchRadius=0;
 float radius()const{return hypotf(fmaxf(front,rear),halfWidth);}
 bool valid()const{
  const float a[]={corridorWidth,front,rear,halfWidth,frontSensor,sideX,sideY,margin,deceleration,reaction,cruise,turnSpeed,track,distanceTolerance,angleTolerance,matchRadius};
  for(float x:a)if(!isfinite(x))return false;
  return corridorWidth>2*(halfWidth+margin)&&front>0&&rear>0&&halfWidth>0&&frontSensor>=0&&frontSensor<=front&&fabsf(sideX)<=fmaxf(front,rear)&&sideY>=0&&sideY<=halfWidth&&margin>0&&deceleration>0&&reaction>0&&cruise>0&&turnSpeed>0&&track>0&&distanceTolerance>0&&distanceTolerance<margin&&angleTolerance>0&&angleTolerance<.25f&&matchRadius>distanceTolerance;
 }
};
struct Ray { float mm=-1;uint32_t stamp=0,sequence=0;bool valid=false;Pose pose;bool noReturn=false; };
struct Input { uint32_t now=0;Pose pose;Ray ranges[3];float leftSpeed=0,rightSpeed=0; };
inline bool fresh(const Ray&r,uint32_t now){return r.valid&&isfinite(r.mm)&&r.mm>=20&&(uint32_t)(now-r.stamp)<300;}
inline float displacement(const Pose&a,const Pose&b){return hypotf(a.x-b.x,a.y-b.y);}
// A conservative clearance correction for motion since each ping. Sonar is a
// beam, not a certified free-space sector; the local straight-wall assumption
// and an unobstructed traversed corridor behind the axle remain explicit.
inline float corrected(const Config&c,const Input&i,int n){
 const Ray&r=i.ranges[n];if(!fresh(r,i.now))return -1;
 float yaw=maze::angleDistance(i.pose.heading,r.pose.heading);
 if(yaw>.18f)return -1;
 float origin=n==0?fabsf(c.frontSensor):hypotf(c.sideX,c.sideY);
 return r.mm-displacement(i.pose,r.pose)-origin*yaw;
}
inline bool turnClear(const Config&c,const Input&i,float rearEvidence){
 float need=c.radius()+c.margin;
 if(rearEvidence<need)return false;
 for(int n=0;n<3;++n){float d=corrected(c,i,n);if(d<0||d+(n==0?c.frontSensor:c.sideY)<=need)return false;}
 return true;
}
struct Belief {
 float wallProbability=.5f;uint32_t last=0,stamp=0;bool sampled=false;Pose evidence;
 void reset(){*this=Belief();}
 void update(const Ray&r,uint32_t now,float wallLimit,float openLimit){
  if(!fresh(r,now)||(sampled&&(int32_t)(r.sequence-last)<=0))return;
  last=r.sequence;
  // At most one likelihood per 75ms while stationary; no duplicate loop evidence.
  if(sampled&&(uint32_t)(r.stamp-stamp)<75&&displacement(r.pose,evidence)<10)return;
  float prior=.9f*wallProbability+.05f; // process uncertainty prevents locked certainty
  float hit=limit(.8f-.6f*(r.mm-wallLimit)/(openLimit-wallLimit),.2f,.8f);
  wallProbability=limit(hit*prior/(hit*prior+(1-hit)*(1-prior)),.02f,.98f);
  evidence=r.pose;stamp=r.stamp;sampled=true;
 }
 bool wall()const{return wallProbability>.8f;}
 bool open()const{return wallProbability<.2f;}
};
class Turn {
 Pose start;float angle=0;uint32_t quietAt=0;bool quiet=false;int corrections=0,lastSign=0;
 public:
 bool done=false,failed=false;
 void begin(float radians,const Pose&p){start=p;angle=radians;done=failed=quiet=false;corrections=lastSign=0;}
 Speeds step(const Config&c,const Input&i){
  if(done||failed)return {};
  float l=i.pose.left-start.left,r=i.pose.right-start.right;
  // Differential wheel feedback explicitly bounds centre drift during a pivot.
  if(fabsf((l+r)*.5f)>c.margin||displacement(start,i.pose)>c.margin){failed=true;return {};}
  float error=angle-(i.pose.heading-start.heading);
  bool stopped=fmaxf(fabsf(i.leftSpeed),fabsf(i.rightSpeed))<1;
  int sign=error>0?1:-1;
  if(fabsf(error)<=c.angleTolerance||(lastSign&&sign!=lastSign)){
   if(!stopped){quiet=false;return {};}
   if(!quiet){quiet=true;quietAt=i.now;return {};}
   if((uint32_t)(i.now-quietAt)<200)return {};
   if(fabsf(error)<=c.angleTolerance){done=true;return {};}
   if(++corrections>2){failed=true;return {};}
   lastSign=0;quiet=false;
  }
  quiet=false;lastSign=sign;
  float remaining=fabsf(error)*c.track*.5f;
  float v=fminf(c.turnSpeed,fminf(sqrtf(2*c.deceleration*remaining),remaining*3));
  float balance=limit((l+r)*2,-v*.6f,v*.6f);
  return {-sign*v-balance,sign*v-balance};
 }
};

// Corridor graph navigation. Junction bearings are approximately orthogonal;
// departures are verified by live ranges. Ambiguous loop closures stop for review.
class Navigator {
 public:
 enum State { IDLE, CORRIDOR, APPROACH, ASSESS, TURNING, VERIFY, CONFIRM, COMPLETE, FAULT } state=IDLE;
 Config config;Belief beliefs[3];const char*reason="WALL_IDLE";
 bool localized=true,paused=true,ownsGraph=false;
 int current=maze::NONE,incoming=maze::NONE,departure=maze::NONE,candidate=maze::NONE;
 explicit Navigator(maze::Graph&g):graph(g){}
 void clear(){graph.clear();state=IDLE;localized=true;paused=true;ownsGraph=false;current=incoming=departure=candidate=maze::NONE;resetBeliefs();reason="WALL_IDLE";}
 // Line guidance may temporarily steer within a corridor. Retain the graph,
 // edge start and travelled distance; do not invent a new node at reacquisition.
 bool resumeAfterLine(const Input&i){
  if(state!=CORRIDOR||!localized)return false;
  heading=i.pose.heading;resetBeliefs();paused=false;reason="WALL_RESUMING_CORRIDOR";return true;
 }
 void pause(){paused=true;}
 void invalidate(){if(ownsGraph&&graph.nodeCount){localized=false;paused=true;reason="WALL_RELOCALIZE_REQUIRED";}}
 bool begin(const Input&i){
  if(!config.valid()){reason="WALL_CALIBRATION_REQUIRED";return false;}
  if(!localized){reason="WALL_RELOCALIZE_REQUIRED";return false;}
  if(state==FAULT||state==COMPLETE||state==CONFIRM)return false;
  if(state==IDLE){
   if(graph.nodeCount){reason="RESET_TAPE_MAP_FIRST";return false;}
   float h=i.pose.heading;current=graph.addNode(i.pose.x,i.pose.y,&h,1);departure=0;
   ownsGraph=true;segmentStart=i.pose.path;heading=h;state=CORRIDOR;exitUntil=i.pose.path;
  }
  paused=false;reason="WALL_WAITING_FOR_RANGES";return true;
 }
 void fail(const char*why){state=FAULT;paused=true;reason=why;}
 bool confirm(int node){
  if(state!=CONFIRM||node!=candidate||!localized)return false;
  if(!commit(node)){fail("WALL_MAP_CONFLICT");return false;}
  state=ASSESS;recorded=true;reason="WALL_CHECKPOINT_CONFIRMED";return true;
 }
 Speeds step(const Input&i){
  if(paused||state==IDLE||state==FAULT||state==COMPLETE||state==CONFIRM)return {};
  float f=corrected(config,i,0),r=corrected(config,i,1),l=corrected(config,i,2);
  if(f<0||r<0||l<0){reason="WALL_RANGE_UNKNOWN";return {};}
  float open=opening();
  for(int n=0;n<3;++n)beliefs[n].update(i.ranges[n],i.now,open*.7f,open);
  float v=fmaxf(fabsf(i.leftSpeed),fabsf(i.rightSpeed));
  float braking=v*config.reaction+v*v/(2*config.deceleration);
  float frontRoom=f+config.frontSensor-config.front-config.margin;
  if(frontRoom<=braking&&state!=TURNING){reason="WALL_BRAKING";if(state==CORRIDOR||state==APPROACH)assess(i);return {};}
  if(state==CORRIDOR){
   bool expected=false;
   int e=graph.nodes[current].ports[departure].edge;
   float remaining=e==maze::NONE?INFINITY:graph.edges[e].length-(i.pose.path-segmentStart);
   if(e!=maze::NONE&&remaining<=config.distanceTolerance)expected=true;
   if(e!=maze::NONE&&!expected&&remaining<=braking){reason="WALL_BRAKING_RETURN";return {};}
   bool sideFeature=e==maze::NONE&&i.pose.path>=exitUntil&&(l>open||r>open);
   float stopAt=config.radius()+2*config.margin-config.frontSensor;
   if(expected||f<=stopAt+braking){assess(i);return {};}
   if(sideFeature){
    approachEnd=i.pose.path+config.sideX+config.radius()+config.margin;
    state=APPROACH;reason="WALL_POSITIONING_JUNCTION";
   }else{reason="WALL_CORRIDOR";float approachSpeed=-config.deceleration*config.reaction+sqrtf(config.deceleration*config.deceleration*config.reaction*config.reaction+2*config.deceleration*fmaxf(0,remaining));
    return corridor(i,f,r,l,fminf(config.cruise,approachSpeed));}
  }
  if(state==APPROACH){
   float remain=approachEnd-i.pose.path;
   if(remain<=fmaxf(config.distanceTolerance,braking)){assess(i);return {};}
   float stopAt=config.radius()+2*config.margin-config.frontSensor;
   if(f<=stopAt+braking){assess(i);return {};}
   reason="WALL_POSITIONING_JUNCTION";
   return corridor(i,f,r,l,fminf(config.cruise,sqrtf(2*config.deceleration*remain)));
  }
  if(state==ASSESS){
   if(!stationary(i)){assessedAt=i.now;reason="WALL_SETTLING";return {};}
   if((uint32_t)(i.now-assessedAt)<200){reason="WALL_SETTLING";return {};}
   // Require every ray to have been acquired after the stop, with enough
   // evidence to classify each non-incoming direction.
   int returnEdge=graph.nodes[current].ports[departure].edge;
   bool returningToRoot=returnEdge>=0&&graph.other(returnEdge,current)==0;
   for(int n=0;n<3;++n){
    if((int32_t)(i.ranges[n].stamp-assessedAt)<=0||(!returningToRoot&&!beliefs[n].wall()&&!beliefs[n].open())){
     if((uint32_t)(i.now-assessedAt)>4000)fail("WALL_AMBIGUOUS_GEOMETRY");
     else reason="WALL_INSPECTING_JUNCTION";return {};
    }
   }
   if(!recorded){if(!checkpoint(i))return {};recorded=true;}
   maze::Choice choice=graph.plan(current,incoming);
   if(choice.port<0&&current!=0)choice.port=homePort();
   if(choice.port<0){state=COMPLETE;paused=true;reason="WALL_MAP_EXPLORED_NOT_GOAL";return {};}
   departure=choice.port;
   float angle=maze::wrap(graph.nodes[current].ports[departure].bearing-i.pose.heading);
   if(fabsf(angle)>config.angleTolerance){
    if(!turnClear(config,i,rearEvidence)){reason="WALL_TURN_CLEARANCE_REQUIRED";return {};}
    turn.begin(angle,i.pose);state=TURNING;reason="WALL_CONTROLLED_PIVOT";
   }else{state=VERIFY;verifyAt=i.now;reason="WALL_VERIFYING_EXIT";}
   return {};
  }
  if(state==TURNING){
   if(!turnClear(config,i,rearEvidence)){reason="WALL_TURN_BLOCKED";return {};}
   Speeds d=turn.step(config,i);
   if(turn.failed){fail("WALL_TURN_DRIFT_OR_OVERSHOOT");return {};}
   if(turn.done){state=VERIFY;verifyAt=i.now;resetBeliefs();reason="WALL_VERIFYING_EXIT";return {};}
   reason="WALL_CONTROLLED_PIVOT";return d;
  }
  if(state==VERIFY){
   if(!stationary(i)){verifyAt=i.now;return {};}
   if((uint32_t)(i.now-verifyAt)<200||(int32_t)(i.ranges[0].stamp-verifyAt)<=0)return {};
   if(f+config.frontSensor<=config.front+config.margin+config.radius()){
    fail("WALL_EXIT_NOT_CLEAR");return {};
   }
   heading=i.pose.heading;segmentStart=i.pose.path;exitUntil=i.pose.path+2*config.radius()+config.margin;
   state=CORRIDOR;reason="WALL_EXIT_VERIFIED";return {};
  }
  return {};
 }
 private:
 maze::Graph&graph;Turn turn;
 float segmentStart=0,heading=0,exitUntil=0,approachEnd=0,rearEvidence=0;
 uint32_t assessedAt=0,verifyAt=0;bool recorded=false;
 Pose pending;float pendingLength=0,pendingBearings[4]={};int pendingCount=0;
 float opening()const{return config.corridorWidth-config.halfWidth-config.sideY+config.margin;}
 bool stationary(const Input&i)const{return fabsf(i.leftSpeed)<1&&fabsf(i.rightSpeed)<1;}
 void resetBeliefs(){for(auto&b:beliefs)b.reset();}
 void assess(const Input&i){state=ASSESS;assessedAt=i.now;recorded=false;rearEvidence=i.pose.path-segmentStart;resetBeliefs();reason="WALL_ASSESSING";}
 Speeds corridor(const Input&i,float f,float r,float l,float speed){
  bool rw=r<opening(),lw=l<opening();
  float error=0;
  // Centring when both walls exist; do not chase an opening as a distant wall.
  float desired=config.corridorWidth*.5f-config.sideY;
  if(rw&&lw)error=(l-r)*.5f;else if(rw)error=desired-r;else if(lw)error=l-desired;
  if(state==APPROACH)error=0; // Preserve entry heading across a partially observed opening.
  float yaw=limit(error*.006f+maze::wrap(heading-i.pose.heading)*1.5f,-.5f,.5f);
  float frontRoom=f+config.frontSensor-config.front-config.margin;
  float cap=-config.deceleration*config.reaction+sqrtf(config.deceleration*config.deceleration*config.reaction*config.reaction+2*config.deceleration*fmaxf(0,frontRoom));
  speed=fminf(speed,cap);
  // Bound the near-term lateral sweep of the front corner; no one-wheel pivot.
  float sideRoom=fminf(l,r)+config.sideY-config.halfWidth-config.margin;
  if(sideRoom<=0){reason="WALL_SIDE_CLEARANCE_REQUIRED";return {};}
  yaw=limit(yaw,-sideRoom/(config.front*.5f+speed*config.reaction),sideRoom/(config.front*.5f+speed*config.reaction));
  float differential=limit(yaw*config.track*.5f,-speed*.4f,speed*.4f);
  return {speed-differential,speed+differential};
 }
 bool checkpoint(const Input&i){
  pending=i.pose;pendingLength=i.pose.path-segmentStart;
  if(pendingLength<=config.distanceTolerance){fail("WALL_START_NEEDS_CORRIDOR");return false;}
  pendingCount=0;
  // Incoming corridor was traversed, not inferred from an unobserved rear ray.
  pendingBearings[pendingCount++]=maze::wrap(i.pose.heading+maze::PI_F);
  if(beliefs[2].open())pendingBearings[pendingCount++]=maze::wrap(i.pose.heading+maze::PI_F/2);
  if(beliefs[0].open())pendingBearings[pendingCount++]=maze::wrap(i.pose.heading);
  if(beliefs[1].open())pendingBearings[pendingCount++]=maze::wrap(i.pose.heading-maze::PI_F/2);
  int edge=graph.nodes[current].ports[departure].edge;
  if(edge!=maze::NONE){
   int expected=graph.other(edge,current);
   if(displacementTo(expected)>config.matchRadius||fabsf(pendingLength-graph.edges[edge].length)>fmaxf(config.matchRadius,graph.edges[edge].length*.25f)||!signature(expected)){
    fail("WALL_RETURN_MISMATCH");return false;
   }
   if(!commit(expected)){fail("WALL_MAP_CONFLICT");return false;}return true;
  }
  candidate=maze::NONE;
  for(int n=0;n<graph.nodeCount;++n)if(displacementTo(n)<=config.matchRadius){
   if(candidate!=maze::NONE){fail("WALL_AMBIGUOUS_JUNCTION");return false;}candidate=n;
  }
  if(candidate!=maze::NONE){
   if(!signature(candidate)){fail("WALL_JUNCTION_CONFLICT");return false;}
   state=CONFIRM;paused=true;reason="WALL_CONFIRM_LOOP_CLOSURE";return false;
  }
  if(graph.nodeCount==maze::Graph::MAX_NODES||graph.edgeCount==maze::Graph::MAX_EDGES){fail("WALL_MAP_FULL");return false;}
  int node=graph.addNode(pending.x,pending.y,pendingBearings,pendingCount);
  if(node<0||!commit(node)){if(node>=0)--graph.nodeCount;fail("WALL_MAP_CONFLICT");return false;}return true;
 }
 int homePort()const{
  float dist[maze::Graph::MAX_NODES];int first[maze::Graph::MAX_NODES];bool used[maze::Graph::MAX_NODES]={};
  for(int n=0;n<graph.nodeCount;++n){dist[n]=INFINITY;first[n]=-1;}dist[current]=0;
  for(int k=0;k<graph.nodeCount;++k){
   int u=-1;for(int n=0;n<graph.nodeCount;++n)if(!used[n]&&(u<0||dist[n]<dist[u]))u=n;
   if(u<0||!isfinite(dist[u]))break;
   if(u==0)return first[u];
   used[u]=true;
   for(int p=0;p<graph.nodes[u].count;++p){auto &port=graph.nodes[u].ports[p];if(port.blocked||port.edge<0)continue;
    int v=graph.other(port.edge,u);int remote=graph.arrivalPort(port.edge,u,p);
    if(v<0||remote<0||graph.nodes[v].ports[remote].blocked)continue;
    float d=dist[u]+graph.edges[port.edge].length;
    if(d<dist[v]){dist[v]=d;first[v]=u==current?p:first[u];}
   }
  }return -1;
 }
 float displacementTo(int n)const{return hypotf(graph.nodes[n].x-pending.x,graph.nodes[n].y-pending.y);}
 bool signature(int n)const{
  // Root has only its departure recorded; returning to it ends this exploration.
  if(n==0)return graph.nearestPort(n,pendingBearings[0],.3f)>=0;
  if(graph.nodes[n].count!=pendingCount)return false;
  for(int k=0;k<pendingCount;++k)if(graph.nearestPort(n,pendingBearings[k],.3f)<0)return false;
  return true;
 }
 bool commit(int n){
  int p=graph.nearestPort(n,pendingBearings[0],.3f);
  if(p<0||!graph.connect(current,departure,n,p,pendingLength))return false;
  current=n;incoming=p;candidate=maze::NONE;return true;
 }
};
}
