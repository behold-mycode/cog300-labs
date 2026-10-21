#include <cassert>
#include <iostream>
#include "../src/MyRobot-V2/MyRobot-V2.ino"
#include "../src/MyRobot-V2/Motor.ino"
#include "../src/MyRobot-V2/Serial.ino"
#include "../src/MyRobot-V2/Safety.ino"
#include "../src/MyRobot-V2/Navigation.ino"
void command(const char* s) { char b[64]; strncpy(b,s,sizeof(b)); processCommand(b); }
void step(int front, int right, uint32_t ms = 50) {
  nowMs += ms; acceptReading(0,front,front > 0 ? front*58UL:0);
  acceptReading(1,right,right > 0 ? right*58UL:0); serviceNavigation(); serviceMotion();
}
void settle(int front, int right) { for(int i=0;i<20;++i) step(front,right); }
void stopped() { assert(pwm[ENA]==0 && pwm[ENB]==0); }
int main() {
  setup(); stopped(); assert(navigationMode==MODE_MANUAL);
  command("MODE FOLLOW"); serviceNavigation(); stopped(); // Cannot start without a reading.
  settle(30,-1); assert(autoLeft==60 && autoRight==60); // right sensor not needed for FOLLOW
  step(20,-1); stopped(); assert(navigationMode==MODE_FOLLOW);
  step(21,-1); stopped(); // Resume deadband avoids threshold chatter.
  step(23,-1); assert(autoLeft==10 && autoRight==10);
  step(10,-1); stopped(); assert(autoLeft>=0 && autoRight>=0);
  settle(40,-1); assert(autoLeft==120);
  step(-1,-1); stopped();
  settle(40,20); nowMs+=STALE_MS; serviceNavigation(); stopped();
  step(151,20); stopped(); // Distant echo not pursued.
  command("STOP"); assert(navigationMode==MODE_MANUAL); stopped();
  command("KP FOLLOW 2.5"); assert(followKp==2.5f);
  command("MODE FOLLOW"); settle(40,20); assert(autoLeft==50);
  command("KP WALL 3"); assert(navigationMode==MODE_MANUAL); stopped();
  command("MODE WALL"); settle(100,30); assert(autoLeft==130 && autoRight==70);
  settle(100,10); assert(autoLeft==70 && autoRight==130);
  settle(100,20); assert(autoLeft==100 && autoRight==100);
  step(10,20); stopped(); assert(cornerTurning && reversalWaiting);
  step(10,20,249); stopped();
  step(10,20,50); assert(autoLeft<0 && autoRight>0);
  settle(30,20); assert(!cornerTurning && autoLeft==100 && autoRight==100);
  step(100,-1); stopped();
  step(-1,20); stopped();
  settle(100,20); step(10,20); step(10,20,TURN_LIMIT_MS);
  stopped(); assert(navigationMode==MODE_MANUAL);
  command("MODE FOLLOW"); settle(30,20);
  command("M 120 120 500"); assert(navigationMode==MODE_MANUAL && pending);
  command("STOP"); stopped();
  command("HELLO"); command("MODE FOLLOW"); step(30,20);
  nowMs+=501; serviceControlTimeout(); stopped(); assert(navigationMode==MODE_MANUAL);
  command("HELLO"); command("KP WALL nan"); stopped();
  assert(wallKp==3.0f); command("KP WALL -1"); assert(wallKp==3.0f);
  command("BASE 999"); assert(wallBasePwm==100);
  // Mode changes always clear pending tests and held drive output.
  command("MODE WALL"); settle(100,20); command("MODE MANUAL"); stopped();
  std::cout << "PASS: P control, distance holds, invalid/stale sensors, corner limit, tuning, manual override\n";
}
