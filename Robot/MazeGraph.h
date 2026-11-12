#pragma once
#include <stdint.h>
#include <math.h>

namespace maze {
constexpr float PI_F = 3.14159265358979323846f;
inline float wrap(float a) { while (a > PI_F) a -= 2*PI_F; while (a <= -PI_F) a += 2*PI_F; return a; }
inline float angleDistance(float a, float b) { return fabsf(wrap(a-b)); }
constexpr int NONE = -1;
struct Port { float bearing = 0; int16_t edge = NONE; bool blocked = false; };
struct Node { float x = 0, y = 0; uint8_t count = 0; Port ports[6]; };
struct Edge { uint8_t a = 0, ap = 0, b = 0, bp = 0; float length = 0; uint16_t traversals = 0; };
struct Choice { int node = NONE, port = NONE; bool exploring = false; };
class Graph {
 public:
  static const int MAX_NODES = 64, MAX_EDGES = 128, MAX_PORTS = 6;
  Node nodes[MAX_NODES]; Edge edges[MAX_EDGES];
  uint16_t nodeCount = 0, edgeCount = 0;
  void clear() { nodeCount = edgeCount = 0; }
  bool validNode(int n) const { return n >= 0 && n < nodeCount; }
  bool validPort(int n, int p) const { return validNode(n) && p >= 0 && p < nodes[n].count; }
  int addNode(float x, float y, const float* headings, int count) {
    if (nodeCount == MAX_NODES || count < 1 || count > MAX_PORTS || !isfinite(x) || !isfinite(y)) return NONE;
    for (int p=0;p<count;++p) if (!isfinite(headings[p])) return NONE;
    int id=nodeCount++; nodes[id]=Node(); nodes[id].x=x; nodes[id].y=y; nodes[id].count=count;
    for (int p=0;p<count;++p) nodes[id].ports[p].bearing=wrap(headings[p]);
    return id;
  }
  int nearestPort(int n, float bearing, float tolerance) const {
    if (!validNode(n)) return NONE;
    int result=NONE;
    for (int p=0;p<nodes[n].count;++p) if (angleDistance(nodes[n].ports[p].bearing,bearing)<=tolerance) {
      if (result != NONE) return NONE; // Ambiguous match, never guess.
      result=p;
    }
    return result;
  }
  bool connect(int a,int ap,int b,int bp,float length) {
    if (!validPort(a,ap)||!validPort(b,bp)||!isfinite(length)||length<=0 || (a==b && ap==bp)) return false;
    int ea=nodes[a].ports[ap].edge, eb=nodes[b].ports[bp].edge;
    if (ea!=NONE || eb!=NONE) {
      if (ea==NONE || ea!=eb) return false;
      Edge &e=edges[ea];
      bool same=(e.a==a&&e.ap==ap&&e.b==b&&e.bp==bp)||(e.a==b&&e.ap==bp&&e.b==a&&e.bp==ap);
      if (!same) return false;
      if (e.traversals < UINT16_MAX) ++e.traversals;
      return true;
    }
    if (edgeCount==MAX_EDGES) return false;
    int id=edgeCount++; Edge &e=edges[id];
    e.a=a; e.ap=ap; e.b=b; e.bp=bp; e.length=length; e.traversals=1;
    nodes[a].ports[ap].edge=nodes[b].ports[bp].edge=id;
    return true;
  }
  int other(int edge,int node) const {
    if (edge<0 || edge>=edgeCount) return NONE;
    return edges[edge].a==node ? edges[edge].b : edges[edge].b==node ? edges[edge].a : NONE;
  }
  int arrivalPort(int edge,int from,int fromPort) const {
    if (edge<0 || edge>=edgeCount) return NONE;
    const Edge &e=edges[edge];
    if(e.a==from && e.ap==fromPort) return e.bp;
    if(e.b==from && e.bp==fromPort) return e.ap;
    return NONE;
  }
  bool frontier(int n,int p) const { return validPort(n,p)&&nodes[n].ports[p].edge==NONE&&!nodes[n].ports[p].blocked; }
  // Local unexplored exits first; otherwise Dijkstra to the nearest frontier.
  // No recursion, no heap, no assumption that the maze is a tree.
  Choice plan(int current,int incoming=NONE) const {
    if (!validNode(current)) return {};
    for(int p=0;p<nodes[current].count;++p) if(p!=incoming&&frontier(current,p)) return {current,p,true};
    float distance[MAX_NODES]; int8_t first[MAX_NODES]; bool used[MAX_NODES]={};
    for(int n=0;n<nodeCount;++n) { distance[n]=INFINITY; first[n]=-1; }
    distance[current]=0;
    for(int step=0;step<nodeCount;++step) {
      int u=NONE;
      for(int n=0;n<nodeCount;++n) if(!used[n]&&(u==NONE||distance[n]<distance[u])) u=n;
      if(u==NONE||!isfinite(distance[u])) break;
      used[u]=true;
      if(u!=current) for(int p=0;p<nodes[u].count;++p) if(frontier(u,p)) return {current,first[u],false};
      for(int p=0;p<nodes[u].count;++p) {
        const Port &port=nodes[u].ports[p];
        if(port.blocked||port.edge==NONE) continue;
        int v=other(port.edge,u); if(v<0) continue;
        int remote=arrivalPort(port.edge,u,p);
        if(remote<0||nodes[v].ports[remote].blocked) continue;
        float next=distance[u]+edges[port.edge].length;
        if(next<distance[v]) { distance[v]=next; first[v]=u==current?p:first[u]; }
      }
    }
    // A root's incoming port can deliberately remain untried until other exits.
    if(frontier(current,incoming)) return {current,incoming,true};
    return {};
  }
};
}
