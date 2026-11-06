#include "firmware_fixture.h"
int main() {
  setup();command("HELLO");command("MODE LINE");tick(10);
  assert(mode==MANUAL&&motorLeft==0&&!strcmp(navigationState,"LINE_NOT_CALIBRATED"));
  command("CONFIG 255 255 1 0 20 10 1");tape(2);command("MODE LINE");tick(250);
  assert(motorLeft==255&&motorRight==255);
  tape(4);command("HEARTBEAT");tick(10);assert(motorLeft==0&&motorRight==255);
  tape(0);tick(10);assert(motorLeft==0&&motorRight==0&&!strcmp(navigationState,"LINE_LOST_USE_MAZE"));
  tape(2);tick(10);assert(motorLeft==255&&motorRight==255);
  tick(501);assert(mode==MANUAL&&motorLeft==0&&motorRight==0);
  command("HELLO");tape(0);command("MODE STOP_ON_TAPE");tick(10);assert(motorLeft==255);
  tape(1);tick(10);assert(mode==MANUAL&&motorLeft==0&&!strcmp(navigationState,"TAPE_REACHED"));
  tape(2);command("MODE STOP_AT_END");tick(10);assert(motorLeft==255);
  tape(0);tick(10);assert(mode==MANUAL&&motorLeft==0&&!strcmp(navigationState,"TAPE_ENDED"));
  command("CONFIG 255 255 1 0 20 10 0");tape(5);command("MODE LINE");tick(250);
  assert(motorLeft==255);tick(10,19);assert(motorLeft==0&&motorRight==0&&!strcmp(navigationState,"FRONT_BLOCKED"));
  tick(10,-1);assert(motorLeft==0&&!strcmp(navigationState,"FRONT_INVALID"));
  command("CONFIG 255 255 1 0 20 10 2");assert(lineTapeLevel==0&&mode==MANUAL);
}
