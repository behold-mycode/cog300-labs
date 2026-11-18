#include "../Robot/LineDrive.h"
#include <cassert>
int main(){
 line::PulseDrive drive;
 auto step=[&](uint32_t t,wall::Speeds d,bool precise,int l=0,int r=0,float heading=0){return drive.step(t,d,255,precise,l,r,heading);};
 auto p=step(0,{80,80},false);assert(p.left==255&&p.right==255);
 p=step(10,{-40,40},true);assert(p.left==0&&p.right==0);
 p=step(129,{-40,40},true);assert(p.left==0);
 p=step(130,{-40,40},true);assert(p.left==-255&&p.right==255);
 // Bridge reversal delay must not consume the pulse before motors turn on.
 p=step(400,{-40,40},true);assert(p.left==-255);
 p=step(410,{-40,40},true,-255,255);assert(p.left==-255);
 p=step(445,{-40,40},true,-255,255);assert(p.left==0&&p.right==0);
 p=step(564,{-40,40},true);assert(p.left==0);
 p=step(565,{-40,40},true);assert(p.left==-255);
 p=step(570,{-40,40},true,-255,255);assert(p.left==-255);
 p=step(575,{-40,40},true,-255,255,.04f);assert(p.left==0); // Encoder limit ends pulse early.
 p=step(580,{80,80},false);assert(p.left==0); // Settle before accelerating again.
 p=step(700,{80,80},false);assert(p.left==255);
 p=step(710,{},true);assert(p.left==0);
 p=step(720,{-40,40},true);assert(p.left==0);
 drive.reset();p=step(730,{80,80},false);assert(p.left==255);
}
