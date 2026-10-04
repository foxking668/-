#include "core.hpp"
#include <limits>
#include <queue>

namespace car2026 {
namespace {
struct HSV { double h,s,v; };
HSV hsv(Pixel p) {
    double r=p.r/255.,g=p.g/255.,b=p.b/255.;
    double hi=std::max({r,g,b}),lo=std::min({r,g,b}),d=hi-lo,h=0;
    if(d>1e-9) {
        if(hi==r) h=std::fmod((g-b)/d,6.);
        else if(hi==g) h=(b-r)/d+2;
        else h=(r-g)/d+4;
        h*=30; if(h<0) h+=180;
    }
    return {h,hi>0 ? d/hi*255 : 0,hi*255};
}
struct Run { int a,b; double center() const { return (a+b)*0.5; } int width() const { return b-a+1; } };
std::vector<Run> runs(const std::vector<uint8_t>& mask,int w,int y,int a,int b) {
    std::vector<Run> result; int start=-1;
    for(int x=a;x<=b;++x) {
        if(mask[y*w+x] && start<0) start=x;
        if(start>=0 && (!mask[y*w+x] || x==b)) {
            const int end=mask[y*w+x] ? x : x-1;
            result.push_back({start,end}); start=-1;
        }
    }
    return result;
}
double pathX(const Path& path,int y,int h,double fallback) {
    if(path.x.empty()) return fallback;
    int i=std::min(int(path.x.size())-1,std::max(0,int(double(y)/h*path.x.size())));
    return path.x[i]>=0 ? path.x[i] : fallback;
}
bool isRing(Stage s) { return s==Stage::RingEntry||s==Stage::RingLap||s==Stage::RingExit; }
}

void measurePath(Path& path) {
    if(path.x.empty()) { path.confidence=0; return; }
    const int h=static_cast<int>(path.x.size());
    auto mean=[&](double ratio) {
        int y=int(ratio*h), radius=std::max(2,h/30), count=0; double total=0;
        for(int i=std::max(0,y-radius);i<std::min(h,y+radius+1);++i)
            if(path.x[i]>=0) { total+=path.x[i]; ++count; }
        return count ? total/count : -1.0;
    };
    double near=mean(.84),middle=mean(.65),far=mean(.45);
    if(near<0 || middle<0) { path.confidence=0; return; }
    if(far<0) far=middle;
    path.lateral=near-.5;
    path.heading=far-near;
    path.curvature=far-2*middle+near;
}

void Vision::resetTracking() { previousBlack_=Path{}; }

Path Vision::trackBlack(int w,int h,Stage stage) {
    Path path; path.x.assign(h,-1);
    const int bottom=int(h*.95), top=int(h*std::max(.35,params_.crop_top_ratio));
    double predicted=pathX(previousBlack_,bottom,h,.5)*w, slope=0;
    int hits=0,lastY=bottom,misses=0;
    for(int y=bottom;y>=top;--y) {
        auto candidates=runs(black_,w,y,int(w*.04),int(w*.96));
        double bestScore=std::numeric_limits<double>::infinity(); int chosen=-1;
        const double prior=pathX(previousBlack_,y,h,-1);
        for(size_t k=0;k<candidates.size();++k) {
            const auto& r=candidates[k];
            if(r.width()<std::max(1.,w*params_.line_width_min_ratio) || r.width()>w*params_.line_width_max_ratio) continue;
            const double dx=r.center()-predicted;
            if(std::abs(dx)>w*params_.line_search_ratio) continue;
            double score=std::abs(dx)+(prior>=0 ? .25*std::abs(r.center()-prior*w):0);
            // On departure and subsequent laps the selected line should continue upwards.
            // At the merge, slope continuity outranks line width/area.
            if(stage==Stage::Depart) score+=.1*std::abs(r.center()-w*.5);
            if(score<bestScore) { bestScore=score; chosen=int(k); }
        }
        if(chosen>=0) {
            const double x=candidates[chosen].center();
            const double newSlope=(x-predicted)/std::max(1,lastY-y);
            slope=.7*slope+.3*clamp(newSlope,-3,3);
            predicted=x+slope; path.x[y]=x/w; ++hits; lastY=y; misses=0;
        } else { predicted=clamp(predicted+slope,0,w-1); ++misses; }
        if(misses>h/8) break;
    }
    path.confidence=double(hits)/std::max(1,bottom-top+1);
    // Only bridge short gaps; long absences remain visible to the controller.
    int previous=-1;
    for(int y=top;y<=bottom;++y) if(path.x[y]>=0) {
        if(previous>=0 && y-previous<=h/10)
            for(int i=previous+1;i<y;++i) path.x[i]=path.x[previous]+(path.x[y]-path.x[previous])*double(i-previous)/(y-previous);
        previous=y;
    }
    measurePath(path);
    if(path.confidence>=params_.line_min_confidence) previousBlack_=path;
    return path;
}

Path Vision::trackRoad(int w,int h,Stage stage) {
    Path path; path.x.assign(h,-1);
    double predicted=.5*w;
    int hits=0,bottom=int(h*.95),top=int(h*.38);
    // White corridor, with narrow black guide-line gaps filled. Never fill blue island/background.
    auto road=white_;
    for(int y=top;y<=bottom;++y) {
        auto sections=runs(white_,w,y,0,w-1);
        for(size_t i=1;i<sections.size();++i) {
            int a=sections[i-1].b+1,b=sections[i].a-1;
            if(b-a+1>w*.13) continue;
            bool blueGap=false;
            for(int x=a;x<=b;++x) if(blue_[y*w+x]) blueGap=true;
            if(!blueGap) for(int x=a;x<=b;++x) road[y*w+x]=1;
        }
    }
    for(int y=bottom;y>=top;--y) {
        auto sections=runs(road,w,y,0,w-1); double best=1e9; int selected=-1;
        for(size_t i=0;i<sections.size();++i) {
            auto r=sections[i]; if(r.width()<w*.12 || r.width()>w*.94) continue;
            // Prefer corridor containing the previous point, then nearest centre.
            double score=std::abs(r.center()-predicted);
            if(predicted>=r.a && predicted<=r.b) score*=.3;
            if(isRing(stage) && y<h*.7) score+=params_.ring_side*(r.center()-w*.5)*.12;
            if(score<best) { best=score; selected=int(i); }
        }
        if(selected<0) continue;
        auto r=sections[selected];
        predicted=.65*r.center()+.35*predicted;
        path.x[y]=predicted/w; ++hits;
    }
    path.confidence=double(hits)/std::max(1,bottom-top+1);
    measurePath(path); return path;
}

std::vector<Blob> Vision::blueBlobs(int w,int h) const {
    std::vector<Blob> result; std::vector<uint8_t> visited(w*h);
    for(int y=int(h*params_.crop_top_ratio);y<h;++y) for(int x=0;x<w;++x) {
        int index=y*w+x; if(!blue_[index]||visited[index]) continue;
        std::vector<int> queue{index}; visited[index]=1;
        int minX=x,maxX=x,minY=y,maxY=y;
        for(size_t head=0;head<queue.size();++head) {
            int current=queue[head],cx=current%w,cy=current/w;
            minX=std::min(minX,cx); maxX=std::max(maxX,cx);
            minY=std::min(minY,cy); maxY=std::max(maxY,cy);
            const int dx[4]={-1,1,0,0},dy[4]={0,0,-1,1};
            for(int i=0;i<4;++i) {
                int nx=cx+dx[i],ny=cy+dy[i];
                if(nx<0||nx>=w||ny<int(h*params_.crop_top_ratio)||ny>=h) continue;
                int ni=ny*w+nx;
                if(blue_[ni]&&!visited[ni]) { visited[ni]=1; queue.push_back(ni); }
            }
        }
        if(queue.size()<4) continue;
        result.push_back({minX,minY,maxX-minX+1,maxY-minY+1,int(queue.size()),(minX+maxX)*.5/w,double(maxY)/h});
    }
    return result;
}

Observation Vision::analyze(const Image& image,Stage stage) {
    Observation o; const int w=image.width,h=image.height;
    if(w<32||h<24||image.pixels.size()!=size_t(w)*h) return o;
    black_.assign(w*h,0);white_.assign(w*h,0);blue_.assign(w*h,0);
    std::vector<uint8_t> barMask(w*h);
    int exposurePixels=0;
    for(int y=0;y<h;++y) for(int x=0;x<w;++x) {
        auto c=hsv(image.at(x,y)); int i=y*w+x;
        black_[i]=(c.v<=params_.black_v_max && c.s<=params_.black_s_max);
        white_[i]=(c.v>=params_.white_v_min && c.s<=params_.white_s_max);
        blue_[i]=(c.h>=params_.blue_h_min && c.h<=params_.blue_h_max && c.s>=params_.blue_s_min && c.v>=params_.blue_v_min);
        barMask[i]=(c.s>=params_.bar_saturation_min && c.v>50 && !blue_[i]);
        if(c.v>35) ++exposurePixels;
    }
    o.frameValid=exposurePixels>w*h*.08;
    if(!o.frameValid) return o;
    o.blackPath=trackBlack(w,h,stage);
    o.roadPath=trackRoad(w,h,stage);
    auto blobs=blueBlobs(w,h);
    for(const auto& b:blobs) {
        const double area=double(b.area)/(w*h),aspect=double(b.w)/b.h;
        bool interior=b.x>2 && b.x+b.w<w-2 && b.y>int(h*params_.crop_top_ratio)+1 && b.y+b.h<h-2;
        int whiteBelow=0,sy=std::min(h-1,b.y+b.h+2);
        for(int x=std::max(0,b.x-3);x<std::min(w,b.x+b.w+3);++x) whiteBelow+=white_[sy*w+x];
        if(interior && area>=params_.cone_min_area_ratio && area<=params_.cone_max_area_ratio &&
           aspect>=.25 && aspect<=1.6 && whiteBelow>=b.w*.5) o.cones.push_back(b);
        // An enclosed large blue island is different from blue background touching image edges.
        int surround=0,total=0;
        for(int x=std::max(0,b.x-4);x<std::min(w,b.x+b.w+4);++x) {
            surround+=white_[std::max(0,b.y-3)*w+x]+white_[sy*w+x]; total+=2;
        }
        if(interior && aspect>.65 && aspect<2.0 && area>.025 && total && double(surround)/total>.55)
            o.ringCandidate=true;
    }
    std::sort(o.cones.begin(),o.cones.end(),[](const Blob& a,const Blob& b){return a.bottomY>b.bottomY;});

    // Multiple dark vertical strips produce several short black runs across road rows.
    // Reject border/background and the single guide line before declaring a crossing.
    int stripeRows=0; double stripeYSum=0;
    for(int y=int(h*.38);y<int(h*.93);y+=2) {
        auto dark=runs(black_,w,y,int(w*.12),int(w*.88));
        std::vector<Run> qualified;
        for(auto r:dark) if(r.width()>=std::max(2,w/100) && r.width()<=w*.075) qualified.push_back(r);
        if(qualified.size()<4) continue;
        int minGap=w,maxGap=0;
        for(size_t k=1;k<qualified.size();++k) { int gap=qualified[k].a-qualified[k-1].b; minGap=std::min(minGap,gap); maxGap=std::max(maxGap,gap); }
        if(minGap>=2 && maxGap<=minGap*3 && qualified.back().b-qualified.front().a>w*.18) {
            ++stripeRows;stripeYSum+=double(y)/h;
        }
    }
    o.stripe=stripeRows>=3; if(o.stripe) o.stripeY=stripeYSum/stripeRows;

    // Raised/lowered crossbar must be calibrated from real footage. This uses colourful
    // non-blue occupancy over a broad horizontal span and neutral dark horizontal bars.
    int roiTop=int(h*params_.bar_roi_top),roiBottom=int(h*params_.bar_roi_bottom);
    int barCount=0,area=0,minX=w,maxX=-1,whiteCount=0;
    for(int y=roiTop;y<roiBottom;++y) for(int x=int(w*.15);x<int(w*.85);++x) {
        ++area;whiteCount+=white_[y*w+x];
        if(barMask[y*w+x]) {++barCount;minX=std::min(minX,x);maxX=std::max(maxX,x);}
    }
    bool horizontal=false;
    for(int y=roiTop;y<roiBottom;++y) for(auto r:runs(black_,w,y,int(w*.15),int(w*.85)))
        if(r.width()>w*params_.bar_span_ratio) horizontal=true;
    o.bar=(area && double(barCount)/area>=params_.bar_pixel_ratio && maxX-minX>w*params_.bar_span_ratio)||horizontal;
    // If the road is not visible and no positive bar evidence exists, absence is unknown.
    o.barObservable=o.bar || (area && double(whiteCount)/area>.12);

    // Detect paired tall blue wall columns with a white opening between them.
    // This is restricted to garage stages by Mission; no blue target globally stops the car.
    std::vector<Run> columns; std::vector<uint8_t> columnMask(w);
    for(int x=int(w*.08);x<int(w*.92);++x) {
        int hits=0;
        for(int y=int(h*.38);y<int(h*.88);++y) hits+=blue_[y*w+x];
        columnMask[x]=hits>h*.28;
    }
    columns=runs(columnMask,w,0,int(w*.08),int(w*.92));
    for(size_t a=0;a<columns.size();++a) for(size_t b=a+1;b<columns.size();++b) {
        auto l=columns[a],r=columns[b];
        if(l.width()>w*.18||r.width()>w*.18||r.a-l.b<w*.20||r.a-l.b>w*.65) continue;
        int whites=0,checks=0;
        for(int y=int(h*.60);y<int(h*.88);y+=3) for(int x=l.b+1;x<r.a;x+=3) { whites+=white_[y*w+x]; ++checks; }
        if(checks && double(whites)/checks>.5) { o.garage=true;o.garageError=(l.b+r.a)*.5/w-.5; }
    }
    return o;
}

Path Vision::avoidCones(const Observation& o,const Image& image) {
    Path path=o.roadPath;
    if(!path.valid()) return path;
    // Keep the path inside white road margins; inflate cones by a tunable clearance.
    // Select the nearest cone, not an average of opposite-side cones.
    if(o.cones.empty()) return path;
    const auto& cone=o.cones.front();
    const double center=pathX(path,int(cone.bottomY*image.height),image.height,.5);
    const double halfWidth=double(cone.w)/(2*image.width)+params_.cone_clearance_ratio;
    const double left=cone.bottomX-halfWidth,right=cone.bottomX+halfWidth;
    const bool passRight=cone.bottomX<center;
    for(int y=int(image.height*.38);y<int(image.height*.95);++y) {
        if(path.x[y]<0) continue;
        const double distance=std::abs(double(y)/image.height-cone.bottomY);
        const double influence=std::exp(-distance*distance/.09);
        const double target=passRight ? std::max(path.x[y],right) : std::min(path.x[y],left);
        const double candidate=path.x[y]+influence*(target-path.x[y]);
        const int x=int(clamp(candidate,0.,.999)*image.width);
        // If clearance cannot fit inside white road, do not invent a route through background.
        if(!white_[y*image.width+x] && !black_[y*image.width+x]) { path.confidence=0;return path; }
        path.x[y]=clamp(candidate,.06,.94);
    }
    measurePath(path); return path;
}
} // namespace car2026
