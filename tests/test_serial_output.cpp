#include "../Robot/SerialQueue.h"
#include <cassert>
#include <string>
int main(){
 serialio::Queue q;for(int n=0;n<1000;++n)for(char c:std::string("DATA,1,LINE,0,0\n"))q.write(c);
 std::string received;while(q.available()){received.push_back(q.front());q.pop();}
 assert(!received.empty()&&received.back()=='\n');
 size_t start=0;while(start<received.size()){auto end=received.find('\n',start);assert(received.substr(start,end-start)=="DATA,1,LINE,0,0");start=end+1;}
 // Oversized lines and saturated queues never leak partial protocol records.
 for(int n=0;n<600;++n)q.write('X');q.write('\n');assert(!q.available());
 for(char c:std::string("OK STOP\n"))q.write(c);received.clear();while(q.available()){received+=q.front();q.pop();}assert(received=="OK STOP\n");
}
