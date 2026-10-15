#include "NewTrack/ConeBoundary.hpp"
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iostream>
using namespace NewTrack::ConeBoundary;
cv::Mat road(int w=80,int h=60) {
    cv::Mat m(h,w,CV_8UC3,cv::Scalar(0,0,0));
    cv::rectangle(m,{w/8,0,w*3/4,h},{230,230,230},-1);return m;
}
void cone(cv::Mat& img,int x) {
    std::vector<cv::Point> p{{x,28},{x-4,43},{x+4,43}};
    cv::fillConvexPoly(img,p,{255,0,0});
}
int main() {
    Settings s;Detector d;
    auto frame=road();cone(frame,26);
    std::vector<int> l(60,10),r(60,69),m(60,39);auto originalL=l,originalR=r;
    auto a=d.update(frame,l,r,m,1000,s);assert(a.detected && !a.active && l==originalL && r==originalR);
    a=d.update(frame,l,r,m,1040,s);assert(a.active && a.side==Side::LEFT && r==originalR);
    assert(l[43]>=34 && m[43]>39);
    for(int y=0;y<60;++y) assert(m[y]==(l[y]+r[y])/2 && l[y]<r[y]);
    l=originalL;r=originalR;m.assign(60,39);
    a=d.update(road(),l,r,m,1080,s);assert(a.active && a.held);
    l=originalL;r=originalR;m.assign(60,39);
    a=d.update(road(),l,r,m,1201,s);assert(!a.active && !a.held && l==originalL && r==originalR);
    d.reset();frame=road();cone(frame,54);l=originalL;r=originalR;m.assign(60,39);
    d.update(frame,l,r,m,2000,s);a=d.update(frame,l,r,m,2040,s);
    assert(a.active && a.side==Side::RIGHT && l==originalL && r[43]<=46 && m[43]<39);
    // A visible opposite cone discards the old hold before confirming the new side.
    frame=road();cone(frame,26);l=originalL;r=originalR;m.assign(60,39);
    a=d.update(frame,l,r,m,2080,s);assert(!a.active && a.side==Side::LEFT);
    a=d.update(road(),l,r,m,2120,s);assert(!a.active && !a.held);
    // Global board rejection survives ROI clipping; blue surroundings must never become a cone.
    d.reset();frame=road();cv::rectangle(frame,{0,0,34,48},{255,0,0},-1);
    a=d.update(frame,l,r,m,3000,s);assert(!a.detected && !a.active);
    frame=road();cv::rectangle(frame,{18,29,10,12},{0,0,255},-1);
    a=d.update(frame,l,r,m,3040,s);assert(!a.detected); // Red object is not the specified blue cone.
    // Prefer the nearest of alternating cones, not a fixed scripted left/right sequence.
    d.reset();frame=road();cone(frame,26);
    std::vector<cv::Point> p{{54,36},{50,50},{58,50}};cv::fillConvexPoly(frame,p,{255,0,0});
    l=originalL;r=originalR;m.assign(60,39);d.update(frame,l,r,m,4000,s);
    a=d.update(frame,l,r,m,4040,s);assert(a.active && a.side==Side::RIGHT);
    // Too little free corridor: reject the entire repair without crossing boundaries.
    d.reset();frame=road();cone(frame,38);l.assign(60,10);r.assign(60,48);m.assign(60,45);
    const auto savedL=l,savedR=r;d.update(frame,l,r,m,5000,s);a=d.update(frame,l,r,m,5040,s);
    assert(a.blocked && !a.active && l==savedL && r==savedR);
    // Resolution-scaled geometry and invalid-input reset.
    d.reset();frame=road(160,120);std::vector<cv::Point> pp{{52,56},{44,86},{60,86}};cv::fillConvexPoly(frame,pp,{255,0,0});
    l.assign(120,20);r.assign(120,139);m.assign(120,79);d.update(frame,l,r,m,6000,s);
    a=d.update(frame,l,r,m,6040,s);assert(a.active && l[86]>=68 && m[86]>79);
    assert(!d.update(cv::Mat(),l,r,m,6080,s).active);
    // A cone straight ahead must be detected before deciding its bypass side.
    d.reset();frame=road();cone(frame,39);l=originalL;r=originalR;m.assign(60,39);
    a=d.update(frame,l,r,m,7000,s);assert(a.detected && !a.confirmed && a.side==Side::CENTER);
    a=d.update(frame,l,r,m,7033,s);assert(a.confirmed && a.active && a.side==Side::CENTER && r[43]<39);
    // Same timestamp is not a second new frame. A gap is not two consecutive observations.
    d.reset();l=originalL;r=originalR;m.assign(60,39);
    d.update(frame,l,r,m,8000,s);a=d.update(frame,l,r,m,8000,s);assert(!a.confirmed);
    a=d.update(frame,l,r,m,8200,s);assert(!a.confirmed && a.confirmations==1);
    a=d.update(frame,l,r,m,8233,s);assert(a.confirmed);
    // Colour candidates still confirm with unknown road, but cannot modify a route or start a turn.
    d.reset();l.assign(60,-1);r.assign(60,-1);m.assign(60,-1);
    d.update(frame,l,r,m,9000,s);a=d.update(frame,l,r,m,9033,s);
    assert(a.detected && a.confirmed && !a.roadValid && !a.active && !a.blocked);
    // Growing near cone: old global 24% width/8% area limits would reject this compact tapered shape.
    d.reset();frame=road();std::vector<cv::Point> nearCone{{40,24},{28,53},{52,53}};
    cv::fillConvexPoly(frame,nearCone,{255,0,0});l=originalL;r=originalR;m.assign(60,39);
    d.update(frame,l,r,m,10000,s);a=d.update(frame,l,r,m,10033,s);
    assert(a.detected && a.confirmed && a.side==Side::CENTER && a.rejectedSize==0);
    std::cout<<"PASS cone boundary: left/right, solid cone, confirmation, hold, side change, board rejection, corridor, scaling\n";
}
