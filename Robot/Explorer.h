#pragma once
#include "WorldModel.h"
namespace maze {
struct Geometry {
  float sensorOffset=0,tapeWidth=0,matchRadius=0,turnClearance=0;
  float distanceTolerance=0,angleTolerance=0;
  bool valid() const {
    return isfinite(sensorOffset)&&isfinite(tapeWidth)&&isfinite(matchRadius)&&isfinite(turnClearance)&&
      isfinite(distanceTolerance)&&isfinite(angleTolerance)&&sensorOffset>tapeWidth&&tapeWidth>0&&
      matchRadius>0&&turnClearance>0&&distanceTolerance>0&&distanceTolerance<tapeWidth&&
      angleTolerance>0&&angleTolerance<atan2f(tapeWidth,sensorOffset);
  }
};
// Local behavior owns movement. Graph information is a proposal only; it never
// bypasses a live tape observation or the motion guard in Navigation.cpp.
class Explorer {
 public:
  enum State { IDLE,TRACK,SETTLE,SEARCH,PROBE_ALIGN,PROBE,VERIFY,RETURN,COMPLETE,NEED_CONFIRM,FAULT } state=IDLE;
  Graph graph;Geometry geometry;Dynamics dynamics;WorldModel world;
  bool paused=true,localized=true,backtracking=false;
  int current=NONE,incoming=NONE,departure=NONE,candidate=NONE;
  const char* reason="MAZE_IDLE";
  void clear() {
    graph.clear();world.reset();state=IDLE;paused=true;localized=true;backtracking=false;
    current=incoming=departure=candidate=NONE;reason="MAZE_IDLE";settled=0;
  }
  void pause() { paused=true;settled=0;requested={}; }
  void interruptObservation() { settled=0; }
  void invalidate() { if(graph.nodeCount) { localized=false;paused=true;reason="RELOCALIZE_REQUIRED"; } }
  bool begin(const Pose& p) {
    if(!geometry.valid()) { reason="GEOMETRY_REQUIRED";return false; }
    if(!dynamics.valid()) { reason="DYNAMICS_REQUIRED";return false; }
    if(!localized) { reason="RELOCALIZE_REQUIRED";return false; }
    if(state==FAULT||state==NEED_CONFIRM||state==COMPLETE) return false;
    if(state==IDLE) {
      float h=p.heading;current=graph.addNode(p.x,p.y,&h,1);departure=0;
      segmentStart=p.path;lastTrack=p;state=TRACK;
    }
    paused=false;settled=0;return true;
  }
  bool observe(const Observation& o) { return world.update(o,geometry.distanceTolerance,geometry.sensorOffset); }
  Drive step(uint32_t now) {
    if(paused||state==IDLE||state==FAULT||state==NEED_CONFIRM||state==COMPLETE) return {};
    if(!world.initialized||(uint32_t)(now-world.latest.stamp)>=100) { reason="OBSERVATION_STALE";return {}; }
    if(!world.fresh) return requested; // Freshness is independently checked by guard.
    world.fresh=false;
    const Pose& p=world.latest.pose;uint8_t tape=world.latest.tape;
    Drive next=decide(p,tape);requested=next;return next;
  }
  bool confirm(int node) {
    if(!localized||state!=NEED_CONFIRM||node!=candidate||!graph.validNode(node)) return false;
    if(!commitCheckpoint(node)) return false;
    state=TRACK;reason="LINE_TRACKING";return true;
  }
  bool turning() const { return state==SEARCH||state==RETURN||state==PROBE_ALIGN; }
  void fail(const char* why) { reason=why;state=FAULT;paused=true;requested={}; }
 private:
  Drive requested;
  Pose lastTrack,anchor,verifyStart,pending;
  float segmentStart=0,entryHeading=0,entryUncertainty=0,target=0,scanLimit=0,pendingExit=0,searchTravel=0,lastSearchHeading=0;
  int preferred=1,leg=0,settled=0;
  bool returning=false,feature=false,probed=false;
  float probeX=0,probeY=0;
  float cone() const { return atan2f(geometry.tapeWidth,geometry.sensorOffset)+fmaxf(0,world.headingUncertainty-entryUncertainty); }
  bool stationary() {
    // Quantization-aware stop evidence, not an elapsed turn duration.
    float threshold=0;
    if(world.speed()<=threshold) ++settled;else settled=0;
    return settled>=3;
  }
  float alignmentTolerance() const { return fmaxf(geometry.angleTolerance,.5f*atan2f(geometry.tapeWidth,geometry.sensorOffset)); }
  Drive turnTo(const Pose& p,float heading) {
    float error=heading-p.heading;
    if(fabsf(error)<=alignmentTolerance()) return {};
    int8_t d=error>0?1:-1;return {(int8_t)-d,d};
  }
  void startRecovery(const Pose& p,bool wide) {
    anchor=p;entryHeading=lastTrack.heading;entryUncertainty=world.headingUncertainty;feature=wide;
    preferred=world.lateral()>=0?1:-1;
    // The graph may nominate a direction, but the sensor must still find it.
    if(graph.validNode(current)&&hypotf(graph.nodes[current].x-p.x,graph.nodes[current].y-p.y)<geometry.matchRadius) {
      Choice c=graph.plan(current,incoming);
      if(c.port>=0) preferred=wrap(graph.nodes[current].ports[c.port].bearing-p.heading)>=0?1:-1;
    }
    probeX=lastTrack.x+geometry.sensorOffset*cosf(lastTrack.heading);
    probeY=lastTrack.y+geometry.sensorOffset*sinf(lastTrack.heading);
    leg=0;returning=false;probed=false;settled=0;state=SETTLE;reason="ASSESSING_TAPE";
  }
  Drive decide(const Pose& p,uint8_t tape) {
    switch(state) {
      case TRACK:
        // A fresh contradictory reading always wins over a confident prior.
        if(tape==0||tape==5||tape==7) { startRecovery(p,tape!=0);return {}; }
        if(tape==2) { lastTrack=p;reason=backtracking?"BACKTRACK_TRACKING":"LINE_TRACKING";return {1,1}; }
        reason=world.lateral()>=0?"LINE_LEFT":"LINE_RIGHT";
        // Outer-only still means directional evidence, not a manufactured node.
        if(tape==4||tape==6) return {0,1};
        return {1,0};
      case SETTLE:
        if(!stationary()) { reason="WAITING_FOR_WHEEL_STOP";return {}; }
        if(tape==2&&!feature) { state=TRACK;reason="TAPE_RESTORED";return {}; }
        if(cone()>=PI_F/2) { fail("HEADING_UNCERTAIN");return {}; }
        scanLimit=PI_F-cone();target=entryHeading+preferred*scanLimit;
        // Place the unwrapped target around the current orientation.
        target=p.heading+wrap(entryHeading-p.heading)+preferred*scanLimit;
        searchTravel=0;lastSearchHeading=p.heading;state=SEARCH;reason="SEARCHING_TAPE";return {};
      case SEARCH: {
        searchTravel+=fabsf(p.heading-lastSearchHeading);lastSearchHeading=p.heading;
        if(searchTravel>3*scanLimit+2*cone()) { fail("SEARCH_MOTION_INCONSISTENT");return {}; }
        float relative=angleDistance(p.heading,entryHeading+PI_F);
        bool outgoing=relative>cone();
        // A broad mask is not a verified onward line. Only a narrow centre
        // observation can become an exit candidate, and it must survive motion.
        if(tape==2&&world.belief[WorldModel::CENTRE]>world.belief[WorldModel::MISSING]&&outgoing && (!feature||angleDistance(p.heading,anchor.heading)>cone())) {
          verifyStart=p;state=VERIFY;settled=0;reason="VERIFYING_TAPE";return {};
        }
        Drive d=turnTo(p,target);
        bool crossed=leg==0?preferred*(target-p.heading)<=geometry.angleTolerance:
                               -preferred*(target-p.heading)<=geometry.angleTolerance;
        if(crossed||(!d.left&&!d.right)) {
          if(leg==0) { ++leg;target-=2*preferred*scanLimit;reason="SEARCHING_OTHER_SIDE";return {}; }
          if(!probed) {
            probed=true;target=p.heading+wrap(entryHeading-p.heading);
            state=PROBE_ALIGN;reason="ALIGNING_OBSERVED_TAPE_POINT";return {};
          }
          // Exhausting a search is not proof of a dead end. Returning is an
          // explicit fallback action, accepted only if incoming tape is found.
          target=p.heading+wrap(entryHeading+PI_F-p.heading);state=RETURN;
          returning=true;reason="CHECKING_RETURN_ROUTE";return {};
        }
        reason=leg?"SEARCHING_OTHER_SIDE":"SEARCHING_TAPE";return d;
      }
      case PROBE_ALIGN: {
        Drive d=turnTo(p,target);
        if(fabsf(target-p.heading)>alignmentTolerance()) return d;
        settled=0;state=PROBE;reason="APPROACHING_OBSERVED_TAPE_POINT";return {};
      }
      case PROBE: {
        float remaining=(probeX-p.x)*cosf(entryHeading)+(probeY-p.y)*sinf(entryHeading);
        float velocity=fabsf((world.leftSpeed+world.rightSpeed)*.5f);
        float braking=velocity*dynamics.reactionSeconds+velocity*velocity/(2*dynamics.deceleration);
        if(remaining<=fmaxf(geometry.distanceTolerance,braking)) {
          anchor=p;feature=false;leg=0;settled=0;state=SETTLE;reason="REASSESSING_FROM_OBSERVED_POINT";return {};
        }
        // This movement is bounded by a tape point actually seen before loss,
        // never by a manufactured junction or a timed forward burst.
        if(remaining>geometry.sensorOffset+geometry.tapeWidth) { fail("TAPE_POINT_UNREACHABLE");return {}; }
        reason="APPROACHING_OBSERVED_TAPE_POINT";return {1,1};
      }
      case RETURN: {
        Drive d=turnTo(p,target);
        if(fabsf(target-p.heading)>alignmentTolerance()) return d;
        if(!(tape&2)||tape==7) { fail("RETURN_ROUTE_UNOBSERVED");return {}; }
        verifyStart=p;settled=0;state=VERIFY;reason="VERIFYING_RETURN";return {};
      }
      case VERIFY:
        if(tape!=2&&tape!=3&&tape!=6) {
          state=SEARCH;reason="CANDIDATE_REJECTED";return {};
        }
        // Do not accept centre tape while wheels are still coasting from a spin.
        if(settled<3 && !stationary()) return {};
        if(p.path-verifyStart.path<geometry.tapeWidth) {
          reason="VERIFYING_CONTINUATION";
          if(tape==6) return {0,1};
          if(tape==3) return {1,0};
          return {1,1};
        }
        // Observed traversal, not sensor mask alone, supplies a route checkpoint.
        pending=anchor;pending.path=p.path;pendingExit=p.heading;
        backtracking=returning;
        checkpoint();
        if(state!=NEED_CONFIRM&&state!=FAULT) { state=TRACK;lastTrack=p;reason="LINE_TRACKING"; }
        return {};
      default:return {};
    }
  }
  void checkpoint() {
    float length=pending.path-segmentStart;
    if(length<=geometry.distanceTolerance) return;
    float incomingBearing=wrap(entryHeading+PI_F);
    candidate=NONE;
    for(int n=0;n<graph.nodeCount;++n) {
      if(hypotf(graph.nodes[n].x-pending.x,graph.nodes[n].y-pending.y)<=geometry.matchRadius) {
        if(candidate!=NONE) { fail("AMBIGUOUS_CHECKPOINT");return; }
        candidate=n;
      }
    }
    int known=graph.nodes[current].ports[departure].edge;
    if(candidate!=NONE && known!=NONE && graph.other(known,current)==candidate) {
      commitCheckpoint(candidate);return;
    }
    if(candidate!=NONE) { state=NEED_CONFIRM;paused=true;reason="CONFIRM_ROUTE_CHECKPOINT";return; }
    if(graph.nodeCount==Graph::MAX_NODES||graph.edgeCount==Graph::MAX_EDGES) { fail("MAP_FULL");return; }
    float bearings[]={incomingBearing,wrap(pendingExit)};
    int count=angleDistance(bearings[0],bearings[1])<cone()?1:2;
    int node=graph.addNode(pending.x,pending.y,bearings,count);
    if(node<0) { fail("MAP_FULL");return; }
    if(!commitCheckpoint(node)) --graph.nodeCount;
  }
  bool commitCheckpoint(int node) {
    int port=graph.nearestPort(node,entryHeading+PI_F,cone());
    int exit=graph.nearestPort(node,pendingExit,cone());
    if(port<0||exit<0||!graph.connect(current,departure,node,port,pending.path-segmentStart)) {
      fail("MAP_CONFLICT");return false;
    }
    current=node;incoming=port;departure=exit;segmentStart=pending.path;candidate=NONE;return true;
  }
};
}
