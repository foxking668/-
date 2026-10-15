#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace NewTrack { namespace TrackPolicy {
struct ProbeSettings {
    int waitMs=3000, settleMs=180, timeoutMs=1100, recoverMs=300, cooldownMs=3500;
    int shiftPx80=3, confirmFrames=3, minRows60=6;
    double leftError=18, rightError=10;
    bool requireOpening=true;
    bool requireWidthGrowth=true;
    int widthGrowthPx80=2;
    int entryConfirmFrames=3, entryStablePx80=5, entryStableRows60=6;
};
struct ProbeResult {
    int phase=0; // 0=idle, 1=wait, 2=left probe, 3=right recovery, 4=cooldown
    bool started=false, accepted=false, rejected=false, unknown=false;
    double deltaX=0;
    double deltaWidth=0;
    int widthRows=0;
    int sampleRows=0;
    int clippedRows=0;
};
class EntranceProbe {
    int phase_=0, expand_=0, narrow_=0, opening_=0;
    int entryConfirm_=0, entryX_=-1, entryY_=-1;
    bool haveClock_=false, pausedPreviously_=false;
    long long phaseAt_=0, gateAt_=0;
    std::vector<int> reference_;
    std::vector<int> rightReference_;
public:
    void reset() { *this=EntranceProbe{}; }
    int phase() const {return phase_;}
    bool controlsSteering() const {return phase_==2 || phase_==3;}
    ProbeResult update(bool gate,bool newGate,bool entrySeen,bool taskInside,bool done,bool enabled,
                       bool paused,const std::vector<int>& left,int width,long long now,const ProbeSettings& s,
                       bool fallbackOpening=false,const std::vector<int>* right=nullptr,
                       int entryX=-1,int entryY=-1) {
        ProbeResult out;
        if(!enabled || done || taskInside) {reset();return out;}
        const bool badShape=width<12 || left.size()<12 || (right && right->size()!=left.size());
        if(paused || badShape) { // Freeze the wait/probe clock during a stop or unusable camera frame.
            if(phase_ && haveClock_) {const long long d=now-lastNow_;phaseAt_+=std::max(0LL,d);gateAt_+=std::max(0LL,d);}
            lastNow_=now;haveClock_=true;pausedPreviously_=true;
            entryConfirm_=expand_=narrow_=opening_=0;out.phase=phase_;return out;
        }
        if(pausedPreviously_ && phase_ && haveClock_) {
            const long long d=std::max(0LL,now-lastNow_);phaseAt_+=d;gateAt_+=d;
        }
        lastNow_=now;haveClock_=true;pausedPreviously_=false;
        if(phase_==0 && gate && newGate) {phase_=1;phaseAt_=gateAt_=now;}
        if(phase_==1 && !gate) reset();
        bool start=false;
        if(phase_==1) {
            const bool candidate=entrySeen || (now-gateAt_>=s.waitMs && (!s.requireOpening || fallbackOpening));
            if(candidate) {
                const int maxX=std::max(1,static_cast<int>(std::lround(s.entryStablePx80*width/80.0)));
                const int maxY=std::max(1,static_cast<int>(std::lround(s.entryStableRows60*left.size()/60.0)));
                const bool positioned=entryX>=0 && entryY>=0;
                const bool stable=!positioned || (entryX_>=0 && entryY_>=0 &&
                    std::abs(entryX-entryX_)<=maxX && std::abs(entryY-entryY_)<=maxY);
                if(!stable || entryConfirm_==0) {entryX_=entryX;entryY_=entryY;entryConfirm_=1;}
                else ++entryConfirm_;
                start=entryConfirm_>=std::max(3,s.entryConfirmFrames);
            } else {entryConfirm_=0;entryX_=entryY_=-1;}
        }
        // This confirmation precedes phase 2 and all steering overrides.
        if(start) {
            phase_=2;phaseAt_=now;reference_=left;expand_=narrow_=opening_=0;out.started=true;
            rightReference_=right?*right:std::vector<int>{};
        }
        if(phase_==2 && now-phaseAt_>=s.settleMs) {
            std::vector<int> differences;
            std::vector<int> widthDifferences;
            const int h=static_cast<int>(left.size());
            if(reference_.size()==left.size() && width>=12) {
                for(int y=h/3;y<h-3;++y)
                    if(reference_[y]>=0 && reference_[y]<width && left[y]>=0 && left[y]<width) {
                        differences.push_back(left[y]-reference_[y]);
                        if(left[y]<=1) ++out.clippedRows;
                        if(right && right->size()==left.size() && rightReference_.size()==left.size() &&
                           rightReference_[y]>reference_[y] && rightReference_[y]<width &&
                           (*right)[y]>left[y] && (*right)[y]<width)
                            widthDifferences.push_back(((*right)[y]-left[y])-(rightReference_[y]-reference_[y]));
                    }
            }
            out.sampleRows=static_cast<int>(differences.size());
            const int minRows=std::max(3,static_cast<int>(std::lround(s.minRows60*h/60.0)));
            const int shift=std::max(1,static_cast<int>(std::lround(s.shiftPx80*width/80.0)));
            out.widthRows=static_cast<int>(widthDifferences.size());
            if(!widthDifferences.empty()) {
                std::sort(widthDifferences.begin(),widthDifferences.end());
                out.deltaWidth=widthDifferences[widthDifferences.size()/2];
            }
            const int widthShift=std::max(1,static_cast<int>(std::lround(s.widthGrowthPx80*width/80.0)));
            const bool grew=!s.requireWidthGrowth || (out.widthRows>=minRows && out.deltaWidth>=widthShift);
            if(out.sampleRows>=minRows) {
                std::sort(differences.begin(),differences.end());out.deltaX=differences[differences.size()/2];
                expand_=out.deltaX<=-shift && grew?expand_+1:0;
                narrow_=out.deltaX>=shift?narrow_+1:0;
                // A late gate may start with the opening already clipped at x=0.
                // Require a current opening AND continuous right edge (entrySeen),
                // several measured border rows and the same consecutive confirmation.
                opening_=entrySeen && out.clippedRows>=minRows?opening_+1:0;
            } else expand_=narrow_=opening_=0;
            if(narrow_>=s.confirmFrames) {
                phase_=3;phaseAt_=now;out.rejected=true;
            } else if(expand_>=s.confirmFrames || opening_>=s.confirmFrames) {
                out.accepted=true;reset();
            } else if(now-phaseAt_>=s.timeoutMs) {
                // No measured expansion is not confirmation. End the bounded probe and resume tracking.
                phase_=4;phaseAt_=now;out.unknown=true;
            }
        }
        if(phase_==3 && now-phaseAt_>=s.recoverMs) {phase_=4;phaseAt_=now;}
        if(phase_==4 && now-phaseAt_>=s.cooldownMs) reset();
        out.phase=phase_;return out;
    }
private:
    long long lastNow_=0;
};
} }
