#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
namespace car2026 { namespace capture {
// Independent implementation of the supplied pei reference's row continuity,
// two-sided contrast and near-weighted black-line target. No race/IMU logic.
struct LineFollowTuning {
    bool enabled=false;
    double kp=.35,kd=.02,preview=.25,filterSeconds=.15,deadbandPx=1;
    double maxCommand=8,slewPerSecond=30,targetX=79.5;
    double cropTop=.30,cropBottom=.95,lostSeconds=.35,acquireSeconds=3;
    int contrast=16,maxWidth=26;
    void validate(double limit=15) const {
        const auto range=[](double v,double lo,double hi) {return std::isfinite(v) && v>=lo && v<=hi;};
        if(!range(kp,0,2) || !range(kd,0,.2) || !range(preview,0,1) || !range(filterSeconds,.02,1) ||
           !range(deadbandPx,0,5) || !range(maxCommand,1,std::min(15.,limit)) || !range(slewPerSecond,1,100) ||
           !range(targetX,40,120) || !range(cropTop,0,.75) || !range(cropBottom,.5,1) || cropBottom-cropTop<.2 ||
           !range(lostSeconds,.05,.5) || !range(acquireSeconds,.5,10) || contrast<8 || contrast>80 || maxWidth<4 || maxWidth>40)
            throw std::runtime_error("Invalid stage_1 line-follow parameters; see PARKING_TUNING.md");
    }
    bool operator==(const LineFollowTuning& o) const {
        return enabled==o.enabled && kp==o.kp && kd==o.kd && preview==o.preview && filterSeconds==o.filterSeconds &&
            deadbandPx==o.deadbandPx && maxCommand==o.maxCommand && slewPerSecond==o.slewPerSecond && targetX==o.targetX &&
            cropTop==o.cropTop && cropBottom==o.cropBottom && lostSeconds==o.lostSeconds && acquireSeconds==o.acquireSeconds &&
            contrast==o.contrast && maxWidth==o.maxWidth;
    }
};
constexpr int lineWidth=160,lineHeight=82;
using LineGray=std::array<unsigned char,lineWidth*lineHeight>;
struct LineObservation {
    bool reliable=false;int rows=0,nearRows=0,gaps=0;
    double target=79.5,nearX=79.5,farX=79.5;
};
class BlackLineTracker {
public:
    void reset() {anchor_=-1;}
    LineObservation analyze(const LineGray& image,const LineFollowTuning& tuning) {
        LineObservation result;double prediction=anchor_>=0 ? anchor_ : tuning.targetX;
        double sum=0,weight=0,nearSum=0,nearCount=0,farSum=0,farCount=0;
        int lastY=-1;double lastX=prediction;int lastSpan=0;
        for(int y=78;y>=14;y-=2) {
            const auto at=[&](int x) {return int(image[size_t(y*lineWidth+x)]);};
            double mean=0;for(int x=3;x<157;x+=3) mean+=at(x);mean/=52;
            const double threshold=std::min(112.,mean-tuning.contrast);
            int chosen=-1,spanChosen=0;double best=1e9;
            for(int x=3;x<157;) {
                if(at(x)>threshold) {++x;continue;}
                const int start=x;double dark=0;
                while(x<157 && at(x)<=threshold) dark+=at(x++);
                const int span=x-start,margin=std::max(2,span/2);
                if(span<2 || span>tuning.maxWidth || start-margin<1 || x+margin>=159) continue;
                dark/=span;double left=0,right=0;
                for(int k=1;k<=margin;++k) {left+=at(start-k);right+=at(x-1+k);}
                if(left/margin-dark<tuning.contrast || right/margin-dark<tuning.contrast) continue;
                const double center=(start+x-1)*.5;
                const int gap=lastY<0 ? 0 : lastY-y;
                const double maxStep=lastY<0 ? 55 : std::min(40.,4.+gap*2.);
                if(std::abs(center-prediction)>maxStep) continue;
                // A short dash gap needs compatible anchors; inferred rows never add confidence.
                if(lastY>=0 && gap>18) continue;
                if(gap>=6 && (std::abs(center-lastX)>4.+gap*1.5 || span>lastSpan*2+2 || lastSpan>span*2+2)) continue;
                const double score=std::abs(center-prediction)*10+span;
                if(score<best) {best=score;chosen=start;spanChosen=span;}
            }
            if(chosen<0) continue;
            const double center=chosen+(spanChosen-1)*.5;
            if(lastY>=0 && lastY-y>=6) ++result.gaps;
            prediction=lastX=center;lastY=y;lastSpan=spanChosen;
            const double w=y>=55 ? 3 : y>=35 ? 2 : 1;
            sum+=center*w;weight+=w;++result.rows;
            if(y>=55) {++result.nearRows;nearSum+=center;++nearCount;}
            if(y<=45) {farSum+=center;++farCount;}
        }
        result.reliable=result.rows>=8 && result.nearRows>=3 && farCount>=3;
        if(weight>0) result.target=sum/weight;
        if(nearCount>0) result.nearX=nearSum/nearCount;
        if(farCount>0) result.farX=farSum/farCount;
        if(result.reliable) {
            if(anchor_>=0 && std::abs(result.nearX-anchor_)>18) result.reliable=false;
            else anchor_=result.nearX;
        }
        return result;
    }
private:
    double anchor_=-1;
};
// dt-based PD in servo command units, not the reference's direct PWM units.
class LineSteeringController {
public:
    void reset() {initialized_=false;lastTime_=lastError_=filtered_=derivative_=command_=0;}
    double update(const LineObservation& line,const LineFollowTuning& tuning,double now) {
        if(!line.reliable || !std::isfinite(now) || (initialized_ && now<=lastTime_))
            throw std::runtime_error("Fresh reliable line and increasing time required");
        const double error=line.target-tuning.targetX+tuning.preview*(line.farX-line.nearX);
        const double dt=initialized_ ? now-lastTime_ : 0;
        if(!initialized_) {filtered_=lastError_=error;initialized_=true;}
        else {
            const double alpha=dt/(tuning.filterSeconds+dt);
            filtered_+=alpha*(error-filtered_);
            const double slope=(filtered_-lastError_)/dt;
            derivative_+=alpha*(slope-derivative_);
        }
        const double e=std::abs(filtered_)<=tuning.deadbandPx ? 0 : filtered_-std::copysign(tuning.deadbandPx,filtered_);
        const double desired=std::clamp(tuning.kp*e+tuning.kd*derivative_,-tuning.maxCommand,tuning.maxCommand);
        // Initial output remains zero: motor start never causes a full-angle step.
        const double step=tuning.slewPerSecond*std::min(dt,.1);
        command_+=std::clamp(desired-command_,-step,step);
        lastError_=filtered_;lastTime_=now;return command_;
    }
    double error() const {return filtered_;}
private:
    bool initialized_=false;double lastTime_=0,lastError_=0,filtered_=0,derivative_=0,command_=0;
};
// Readiness persists across frames; neither acquisition nor corrections reset the motor timer.
class LineFollowSession {
public:
    BlackLineTracker tracker;LineSteeringController controller;
    void reset() {tracker.reset();controller.reset();waiting_=false;count_=0;first_=lastValid_=waitStart_=0;}
    void wait(double now) {reset();waiting_=true;waitStart_=now;}
    void observe(const LineObservation& line,double now) {
        if(line.reliable) {if(!count_) first_=now;++count_;lastValid_=now;}
        else {count_=0;first_=0;}
    }
    bool ready() const {return count_>=3 && lastValid_-first_>=.1;}
    bool expired(double now,const LineFollowTuning& tuning) const {return waiting_ && now-waitStart_>=tuning.acquireSeconds;}
    bool lost(double now,const LineFollowTuning& tuning) const {return now-lastValid_>=tuning.lostSeconds;}
    bool waiting() const {return waiting_;}
    void started() {waiting_=false;controller.reset();}
private:
    bool waiting_=false;unsigned count_=0;double first_=0,lastValid_=0,waitStart_=0;
};
}}
