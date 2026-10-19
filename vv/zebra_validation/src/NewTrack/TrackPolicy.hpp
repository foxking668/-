#pragma once

namespace NewTrack { namespace TrackPolicy {

struct ZebraDecision {bool justArmed=false,expired=false,triggered=false,slow=false;};
class ZebraConfirmation {
    int far_=0,near_=0,strong_=0;
    bool armed_=false,seen_=false;
    long long armedAt_=0;
public:
    bool farArmed() const {return armed_;}
    int farFrames() const {return far_;}
    int nearFrames() const {return near_;}
    int strongFrames() const {return strong_;}
    void reset() {*this=ZebraConfirmation{};}
    ZebraDecision update(bool far,bool near,bool strong,long long now,bool allowed,
                         bool requireFar,int farFrames,int nearFrames,int fallbackFrames,int delayMs,int holdMs) {
        ZebraDecision out;
        if(seen_) return out;
        if(!allowed) {far_=near_=strong_=0;armed_=false;return out;}
        if(!armed_) {
            far_=far?far_+1:0;
            if(far_>=farFrames) {armed_=true;armedAt_=now;far_=0;out.justArmed=true;}
        }
        if(armed_ && now-armedAt_>holdMs) {armed_=false;near_=0;out.expired=true;}
        const bool candidate=near && (!requireFar || (armed_ && now-armedAt_>=delayMs));
        near_=candidate?near_+1:0;
        strong_=strong?strong_+1:0;
        out.slow=armed_ || strong_>0;
        if(near_>=nearFrames || (fallbackFrames>0 && strong_>=fallbackFrames)) {
            seen_=true;armed_=false;out.triggered=true;out.slow=false;
        }
        return out;
    }
};

class ForkConfirmation {
    int frames_=0;
    bool armed_=false;
    long long lastSeen_=0;
public:
    void reset() {*this=ForkConfirmation{};}
    bool update(bool visible,bool near,long long now,bool allowed,int frames,int holdMs) {
        if(!allowed) {reset();return false;}
        if(armed_ && now-lastSeen_>holdMs) reset();
        frames_=visible?frames_+1:0;
        if(visible) lastSeen_=now;
        if(frames_>=frames) armed_=true;
        return armed_ && near;
    }
    bool armed() const {return armed_;}
};

struct MarkerLatchResult {
    bool armed=false, justArmed=false, expired=false;
    int confirmed=0;
};

class MarkerLatch {
    bool armed_=false, requireFresh_=false;
    int confirmed_=0;
    long long armedAt_=0;
    // 【修正2】暂停补偿状态：把"暂停期间流逝的时间"从时间轴里扣掉，
    // 这样长暂停不会把已经锁存的门控按墙钟差提前判过期。
    long long lastNow_=0;
    bool lastPaused_=false;
    long long pausedTotal_=0;
public:
    void reset() { *this=MarkerLatch{}; }
    void reject() {armed_=false;confirmed_=0;requireFresh_=true;}
    bool stalled() const {return lastPaused_;}
    MarkerLatchResult update(bool observed,long long now,bool enabled,bool allowed,
                             bool taskActive,bool completed,int frames,int holdMs,
                             bool paused=false) {
        MarkerLatchResult out;
        if(!enabled || completed) {reset();return out;}

        // ---- 【修正2】暂停补偿 ----
        if(paused) {
            // 暂停中：只记录时间戳，不推进确认计数、不产生 justArmed、不判过期。
            lastNow_=now;
            lastPaused_=true;
            out.armed=armed_;
            out.confirmed=confirmed_;
            return out;
        }
        if(lastPaused_) {
            // 恢复后的第一帧：把暂停期间的墙钟时长记入 pausedTotal_，
            // 后续统一用 effNow = now - pausedTotal_ 作为"有效时间轴"。
            if(lastNow_>0) {
                const long long gap=now-lastNow_;
                if(gap>0) pausedTotal_+=gap;
            }
            lastPaused_=false;
        }
        lastNow_=now;
        const long long effNow=now-pausedTotal_;

        if(taskActive) {out.armed=armed_;out.confirmed=confirmed_;return out;}
        if(!allowed) {reset();return out;}
        if(armed_ && effNow-armedAt_>=holdMs) {
            armed_=false;confirmed_=0;requireFresh_=true;out.expired=true;
        }
        if(!observed) {confirmed_=0;requireFresh_=false;}
        if(!armed_ && !requireFresh_ && observed) {
            ++confirmed_;
            if(confirmed_>=frames) {armed_=true;armedAt_=effNow;out.justArmed=true;}
        }
        out.armed=armed_;out.confirmed=confirmed_;return out;
    }
};

inline bool afterZebraResume(long long now,long long stopEnd,int resumeGuardMs) {
    return stopEnd<=0 || now>=stopEnd+resumeGuardMs;
}

struct CrossingResult { bool ready=false, detected=false;int confirmed=0; };
class CrossingCounter {
    int confirmed_=0;
public:
    void reset() {confirmed_=0;}
    CrossingResult update(bool sequenceReady,bool nearVisible,long long now,
                          long long stopEnd,int resumeGuardMs,int requiredFrames) {
        CrossingResult out;
        out.ready=sequenceReady && afterZebraResume(now,stopEnd,resumeGuardMs);
        confirmed_=out.ready && nearVisible?confirmed_+1:0;
        out.confirmed=confirmed_;
        out.detected=confirmed_>=requiredFrames;
        if(out.detected) confirmed_=0;
        return out;
    }
};

} }
