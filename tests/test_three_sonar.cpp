#include "firmware_fixture.h"
int main(){
 setup();assert(!leftSonarEnabled());assert(modes[11]==INPUT);
 // D11 must never be driven against a connected sonar ECHO, including at boot.
 assert(configureSonars(true,25));assert(leftSonarEnabled());
 for(int n=0;n<3;++n){
  nowMs+=25;serviceSensor();assert(pingPending&&activeSensor==n);
  levels[echoPins[n]]=1;captureEcho(n);extraUs+=5800;
  levels[echoPins[n]]=0;captureEcho(n);serviceSensor();
  auto r=sensorReading(n);assert(r.status==RANGE_VALID&&r.cm==100);
 }
 assert(!configureSonars(true,10));
 // A stuck or missing echo is not an opening; next sensor still gets its slot.
 nowMs+=25;serviceSensor();assert(pingPending);nowMs+=46;serviceSensor();
 assert(sensorReading(0).status==RANGE_INVALID);
 assert(configureSonars(false,70));assert(!leftSonarEnabled());
 assert(sensorReading(2).status==RANGE_INVALID);assert(modes[11]==INPUT);
}
