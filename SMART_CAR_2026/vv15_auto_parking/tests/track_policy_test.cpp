#include "NewTrack/TrackPolicy.hpp"
#include "NewTrack/Roundabout.hpp"
#include <cstdlib>
#include <iostream>

using namespace NewTrack;
int checks=0;
void verify(bool ok,int line) {++checks;if(!ok){std::cerr<<"Failed at "<<line<<"\n";std::abort();}}
#define check(x) verify((x),__LINE__)

int main() {
    TrackPolicy::MarkerLatch marker;
    auto m=[&](bool seen,long long now,bool allowed=true,bool active=false,bool done=false,bool on=true){
        return marker.update(seen,now,on,allowed,active,done,2,7000);
    };
    check(!m(true,1000,false).armed);check(!m(true,1200,false).armed);
    check(!m(true,3000).armed);check(m(true,3040).justArmed);
    check(m(false,4000).armed); // Hold beyond visibility to keep the car slow before the corner.
    check(m(true,10040).expired);check(!m(true,10080).armed); // Stale observation cannot instantly re-arm.
    m(false,10120);m(true,10160);check(m(true,10200).justArmed);
    check(m(false,25000,true,true).armed); // Once a real task starts, marker expiry does not abort it.
    check(!m(true,25040,true,false,true).armed); // Completion releases the speed latch.
    marker.reset();m(true,30000);check(m(true,30040).armed);
    check(!m(true,30080,true,false,false,false).armed); // Hot disable.

    TrackPolicy::CrossingCounter cross;
    auto y=[&](long long now,bool ready=true,bool near=true,long long stopEnd=4200){
        return cross.update(ready,near,now,stopEnd,1200,3);
    };
    for(int now=1000;now<4200;now+=40) {const auto r=y(now);check(!r.ready&&!r.detected&&r.confirmed==0);}
    for(int now=4200;now<5400;now+=40) check(!y(now).ready);
    check(!y(5400,true,false).detected); // Far Y is not a near Y.
    check(y(5440).confirmed==1);check(y(5480).confirmed==2);
    check(y(5520).detected);
    check(y(5600).confirmed==1);check(y(5640,false).confirmed==0);
    check(y(5680).confirmed==1);check(y(5700,true,true,9000).confirmed==0); // A new stop discards partial confirmation.
    check(!y(10199,true,true,9000).detected);
    check(y(10200,true,true,9000).confirmed==1);
    check(!TrackPolicy::afterZebraResume(4199,4200,1200));
    check(TrackPolicy::afterZebraResume(5400,4200,1200));
    check(TrackPolicy::afterZebraResume(1000,0,1200)); // Zebra missed: do not deadlock the whole race.

    // Ring geometry cannot start without a confirmed marker, but starts on the same geometry after arming.
    Roundabout::Controller ring;Roundabout::Settings settings;
    std::vector<int> left(60,20),right(60);
    for(int row=0;row<60;++row) {
        right[row]=43+row/2;
        if(row<10||(row>=29&&row<=43)) left[row]=0;
        else if(row<=28) left[row]=26-std::abs(row-18);
    }
    marker.reset();long long now=10000;
    for(int i=0;i<10;++i) check(ring.update(left,right,80,now+i*40,false,true,settings).stage==0);
    const auto first=m(true,11000);check(!first.armed);
    const auto second=m(true,11040);check(second.armed);
    check(ring.update(left,right,80,11040,second.armed,true,settings).stage==0);
    check(ring.update(left,right,80,11080,true,true,settings).stage==1);
    std::cout<<"PASS "<<checks<<" marker/zebra/near-crossing policy checks.\n";
}
