#include "../Robot/LineDrive.h"
#include <cassert>
int main(){
 auto p=lineDrive({80,80},255);assert(p.left==240&&p.right==240);
 p=lineDrive({1,1},255);assert(p.left==240&&p.right==240);
 p=lineDrive({-80,-80},255);assert(p.left==-255&&p.right==-255);
 p=lineDrive({-40,40},255);assert(p.left==-255&&p.right==255);
 p=lineDrive({32,128},255);assert(p.left>=230&&p.left<p.right&&p.right==255);
 p=lineDrive({0,80},255);assert(p.left==0&&p.right==255);
 p=lineDrive({},255);assert(p.left==0&&p.right==0);
 p=lineDrive({80,80},180);assert(p.left==180&&p.right==180);
}
