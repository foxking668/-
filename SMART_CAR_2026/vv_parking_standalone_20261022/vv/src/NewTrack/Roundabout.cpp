#include "Roundabout.hpp"

#include <algorithm>
#include <cmath>

namespace NewTrack { namespace Roundabout {
namespace {
int scaled(int value, int size, int base) {
    return std::max(1, static_cast<int>(std::lround(value * double(size) / base)));
}
int clamp(int x, int width) { return std::max(0, std::min(width - 1, x)); }
bool valid(int x, int width) { return x >= 0 && x < width; }
bool real(int x, int width, bool left) {
    const int margin=scaled(1,width,80);
    return valid(x, width) && (left ? x > margin : x < width - 1 - margin);
}
struct Break { Point near, far; };

std::vector<Break> contourBreaks(const std::vector<Point>& input,int w,int h,const Settings& s) {
    // A row maximum loses horizontal/backward growth. Keep the ordered boundary
    // path (Moore eight-neighbor in NewTrack). Discard disconnected wrap fragments.
    std::vector<Point> path;
    std::size_t begin=0,bestBegin=0,bestLength=0;
    for(std::size_t i=1;i<=input.size();++i) {
        const bool cut=i==input.size() || std::abs(input[i].x-input[i-1].x)>1 ||
                                               std::abs(input[i].y-input[i-1].y)>1;
        if(cut) {
            if(i-begin>bestLength) {bestBegin=begin;bestLength=i-begin;}
            begin=i;
        }
    }
    if(bestLength<8) return {};
    path.assign(input.begin()+bestBegin,input.begin()+bestBegin+bestLength);
    if(path.front().y<path.back().y) std::reverse(path.begin(),path.end());
    const int jump=scaled(s.jumpPx80,w,80),sep=scaled(s.separationRows60,h,60);
    const int window=std::max(10,jump*3);
    std::vector<Break> out;
    for(int i=0;i<static_cast<int>(path.size());++i) {
        const Point a=path[i];
        if(!real(a.x,w,true) || a.y<2 || a.y>=h-2) continue;
        for(int j=i+1;j<static_cast<int>(path.size()) && j<=i+window;++j) {
            const Point b=path[j];const int dx=a.x-b.x,dy=b.y-a.y;
            if(dx<jump || dx<2*std::abs(dy)) continue;
            Point near=a;
            for(int q=i+1;q<=j;++q) if(path[q].x>=near.x) near=path[q];
            if(!out.empty() && out.back().near.y-near.y<sep) break;
            Point far;
            int k=j;
            // Follow this opening to its outside extreme, then its horizontal return.
            int border=-1;
            for(int q=j;q<static_cast<int>(path.size()) && q<=j+window*5;++q)
                if(path[q].x<=scaled(1,w,80)) {border=q;break;}
            if(border>=0) k=border;
            else while(k+1<static_cast<int>(path.size()) && path[k+1].x<=path[k].x && k<i+window*6) ++k;
            while(k+1<static_cast<int>(path.size()) && path[k+1].x<=scaled(1,w,80) && k<i+window*6) ++k;
            const Point outside=path[k];
            for(int q=k+1;q<static_cast<int>(path.size()) && q<=k+window*4;++q) {
                const Point v=path[q];
                if(far.valid() && q>=k+4 &&
                   std::abs(v.x-path[q-4].x)<2*std::abs(v.y-path[q-4].y)) break;
                if(v.y<near.y && v.x-outside.x>=jump &&
                   v.x-outside.x>=2*std::abs(v.y-outside.y)) {
                    if(!far.valid() || v.x>far.x) far=v;
                } else if(far.valid() && v.y<far.y-2) break;
            }
            out.push_back({near,far});i=k;break;
        }
    }
    return out;
}

// Each outward jump or run of missing/border rows is one break, never one per pixel.
std::vector<Break> breaks(const std::vector<int>& b, int w, bool left, const Settings& s) {
    std::vector<Break> out;
    const int h = static_cast<int>(b.size());
    const int gap = scaled(s.minGapRows60, h, 60);
    const int sep = scaled(s.separationRows60, h, 60);
    const int jump = scaled(s.jumpPx80, w, 80);
    auto add = [&](Point near, Point far) {
        if (out.empty() || out.back().near.y - near.y >= sep) out.push_back({near,far});
    };
    for (int y = h - 3; y >= 2; --y) {
        if (!real(b[y], w, left)) continue;
        if (!real(b[y-1], w, left)) {
            int top = y-1;
            while (top >= 0 && !real(b[top], w, left)) --top;
            if (y-top-1 >= gap) {
                Point far = top >= 0 ? Point{b[top],top} : Point{};
                // The left side leaving the image is meaningful even if its upper part is missing.
                const bool outward = left || !far.valid() || far.x-b[y] >= jump;
                if (outward) add({b[y],y},far);
            }
            y = top + 1;
        } else {
            const int d = left ? b[y]-b[y-1] : b[y-1]-b[y];
            // Reject a single pixel spike by checking the real samples on both sides.
            if (d >= jump && y+1 < h && y >= 2 &&
                real(b[y+1],w,left) && real(b[y-2],w,left) &&
                std::abs(b[y+1]-b[y]) < jump && std::abs(b[y-1]-b[y-2]) < jump)
                add({b[y],y},{b[y-1],y-1});
        }
    }
    return out;
}

Point convexMid(const std::vector<int>& b, int w, Point down, Point up, const Settings& s) {
    const int radius = scaled(3, static_cast<int>(b.size()), 60);
    const int jump = scaled(s.jumpPx80,w,80);
    Point best;
    for (int y = down.y-radius; y >= up.y+radius; --y) {
        if (!real(b[y],w,true)) continue;
        // Plateau maxima are accepted; both flanks must fall toward the outside.
        int below = -1, above = -1;
        for (int d = radius; d <= radius*3; ++d) {
            if (y+d < down.y && valid(b[y+d],w)) below = below<0?b[y+d]:std::min(below,b[y+d]);
            if (y-d >= 0 && valid(b[y-d],w)) above = above<0?b[y-d]:std::min(above,b[y-d]);
        }
        if (below >= 0 && above >= 0 && b[y]-below >= std::max(1,jump/2) &&
            b[y]-above >= std::max(1,jump/2) && (!best.valid() || b[y]>best.x)) best={b[y],y};
    }
    return best;
}

bool supported(const std::vector<int>& edge, int from, int count, int direction,
               int w, bool left, int maxStep) {
    for(int n=0;n<count;++n) {
        const int y=from+n*direction;
        if(y<0 || y>=static_cast<int>(edge.size()) || !real(edge[y],w,left)) return false;
        if(n && std::abs(edge[y]-edge[y-direction])>maxStep) return false;
    }
    return true;
}

// 【修复1】允许贴边的支撑检查，专供低相机入口的"上拐点"使用。
//
// 原因：环岛入口的左侧凸弧上方本来就回到画面左缘（x≈0）。real() 会排除贴边，
// 导致 supported() 在凸弧上方必然失败 -> upper 永远找不到 -> 低相机入口不触发。
// 实车 raw_left 的真实形态是：贴边(0..30) -> 凸弧(31..38: 4,5,5,4,4,4,3,2) -> 贴边(39..50)。
//
// 这里仍然是"支撑"检查：连续 count 行坐标有效、且相邻跳变不超过 maxStep。
// 没有取消支撑条件，只是把"贴边不是边界"这个前提按真实画面修正。
bool supportedAllowEdge(const std::vector<int>& edge, int from, int count, int direction,
                        int w, int maxStep) {
    for(int n=0;n<count;++n) {
        const int y=from+n*direction;
        if(y<0 || y>=static_cast<int>(edge.size()) || !valid(edge[y],w)) return false;
        if(n && std::abs(edge[y]-edge[y-direction])>maxStep) return false;
    }
    return true;
}

void detectLowCameraEntry(Evidence& e, const std::vector<int>& left,
                          const std::vector<int>& right, int w, const Settings& s) {
    if(!s.lowCameraEntryEnabled || !e.usable || e.down.valid()) return;
    const int h=static_cast<int>(left.size()),nearEnd=h-1-scaled(6,h,60);
    const int support=std::max(3,scaled(s.lowCameraSupportRows60,h,60));
    const int clipped=std::max(3,scaled(s.lowCameraLeftClippedRows60,h,60));
    const int jump=scaled(s.jumpPx80,w,80),smooth=std::max(1,jump/2);
    const int margin=scaled(1,w,80),sep=std::max(support+1,scaled(s.separationRows60,h,60));
    if(nearEnd-clipped+1<h*2/3) return;
    // Missing rows cannot prove that the left road extends outside the camera.
    for(int y=nearEnd-clipped+1;y<=nearEnd;++y)
        if(!valid(left[y],w) || left[y]>margin) return;

    // Require a supported convex island arc, with actual pixels on both flanks.
    // This remains separate from the standard three-corner detector.
    Point mid,up;
    for(int y=h/4;y<=h*2/3 && y+sep<=nearEnd;++y) {
        if(!real(left[y],w,true) || left[y]>=w/2 || y-sep<0 ||
           !valid(left[y-sep],w) || !valid(left[y+sep],w) ||
           left[y]-left[y-sep]<smooth || left[y]-left[y+sep]<smooth ||
           !supported(left,y,support,-1,w,true,smooth) ||
           !supported(left,y,support,1,w,true,smooth)) continue;
        Point upper;
        for(int q=y-sep;q>=h/6;--q) {
            // 【修复2】凸弧上方回到画面左缘时 x≈0，用 real() 会整体排除，
            // 导致 upper 永远找不到。改用允许贴边的支撑检查（仍要求连续 support 行
            // 有效且跳变受限），没有取消支撑条件。
            if(left[y]-left[q]>=smooth && left[q]<w/2 &&
               supportedAllowEdge(left,q,support,1,w,smooth)) {upper={left[q],q};break;}
        }
        if(upper.valid() && (!mid.valid() || left[y]>mid.x)) {mid={left[y],y};up=upper;}
    }
    if(!mid.valid() || !up.valid() || mid.y<=up.y) return;
    e.lowCameraMid=mid;e.lowCameraUp=up;

    // A right opening is a sustained OUTWARD excursion, with real, smooth
    // boundary samples on BOTH ends. Unknown gaps and isolated spikes fail.
    //
    // 【修复3】右侧开口接受两种真实形态（OR），因为两种都在实车/合成赛道上出现：
    //   形态A 局部凸出：边界先向外鼓出再回到原位置（合成 fixture，right 60->79->60）
    //   形态B 持续外扩：边界一路向右延伸不回退（实车环岛入口，raw_right 36->73）
    // 两种都仍要求：两端真实支撑 + 中间每行有效 + 跨度>=minimum + 位移量>=jump。
    // 没有放宽门槛：仍然拒绝单像素凸起、断行、全白无支撑和非单调回落。
    const int minimum=std::max(4,scaled(s.lowCameraOpeningRows60,h,60));
    for(int a=h/5;a+minimum+support<=nearEnd;++a) {
        if(!supported(right,a-1,support,-1,w,false,smooth) || right[a-1]<w/2) continue;
        const int maxEnd=std::min(nearEnd-support,a+h/3-1);
        for(int b=a+minimum-1;b<=maxEnd;++b) {
            if(!supported(right,b+1,support,1,w,false,smooth) || right[b+1]<w/2) continue;
            bool outward=false;
            // 形态A：局部凸出（相对两端插值线向外鼓出）
            {
                bool ok=true;
                for(int y=a;y<=b;++y) {
                    const double expected=right[a-1]+double(right[b+1]-right[a-1])*(y-a+1)/(b-a+2);
                    if(!valid(right[y],w) || right[y]-expected<jump) {ok=false;break;}
                }
                if(ok) outward=true;
            }
            // 形态B：持续外扩（不回落到起点以下，且整体位移达到阈值）
            if(!outward && right[b+1]-right[a-1]>=jump) {
                bool ok=true;
                for(int y=a;y<=b;++y) {
                    if(!valid(right[y],w) || right[y]<right[a-1]) {ok=false;break;}
                }
                if(ok) outward=true;
            }
            if(!outward) continue;
            e.lowCameraEntry=true;e.rightEntry={right[b+1],b+1};
            e.virtualDown={0,nearEnd};
            e.rightOpeningRows=b-a+1;
            return;
        }
    }
}

void clearAbove(std::vector<int>& b, int y) {
    std::fill(b.begin(), b.begin()+std::max(0,std::min(y,static_cast<int>(b.size()))), -1);
}

// Least squares x=a*y+b. With two anchors this is their exact connecting line.
bool fitAnchors(std::vector<int>& b, const std::vector<Point>& points, int width) {
    if (points.size()<2 || width<2) return false;
    double sy=0,sx=0,syy=0,syx=0;
    int lo=static_cast<int>(b.size()),hi=-1;
    for (const Point p:points) {
        if (!p.valid() || p.y>=static_cast<int>(b.size()) || p.x>=width) return false;
        sy+=p.y; sx+=p.x; syy+=double(p.y)*p.y; syx+=double(p.y)*p.x;
        lo=std::min(lo,p.y);hi=std::max(hi,p.y);
    }
    const double n=static_cast<double>(points.size()),den=n*syy-sy*sy;
    if (std::abs(den)<1e-9) return false;
    const double a=(n*syx-sy*sx)/den,b0=(sx-a*sy)/n;
    for (int y=lo;y<=hi;++y) b[y]=clamp(static_cast<int>(std::lround(a*y+b0)),width);
    return true;
}

Point exitTarget(const std::vector<int>& right, Point corner, int w, const Settings& s) {
    const int h=static_cast<int>(right.size()),targetY=h/3;
    if (s.exitTargetXPercent>0) return {clamp(w*s.exitTargetXPercent/100,w),targetY};
    double sy=0,sx=0,syy=0,syx=0;int n=0;
    // Fit the intact right edge BELOW the corner; never use the outgoing branch above it.
    for (int y=corner.y;y<h-2 && y<=corner.y+std::max(6,h/4);++y) {
        if (!real(right[y],w,false)) continue;
        ++n;sy+=y;sx+=right[y];syy+=double(y)*y;syx+=double(y)*right[y];
    }
    const double den=n*syy-sy*sy;
    int x=corner.x;
    if(n>=3 && std::abs(den)>1e-9) {
        const double a=(n*syx-sy*sx)/den,b=(sx-a*sy)/n;
        x=static_cast<int>(std::lround(a*targetY+b));
    }
    return {clamp(x,w),targetY};
}
}

bool fitSegment(std::vector<int>& bounds, Point a, Point b, int width) {
    return fitAnchors(bounds,{a,b},width);
}

Evidence detect(const std::vector<int>& left, const std::vector<int>& right, int w, const Settings& s,
                const std::vector<Point>& leftContour, const std::vector<Point>& rightContour) {
    Evidence e;
    const int h=static_cast<int>(left.size());
    if (h<12 || w<12 || right.size()!=left.size()) return e;
    int realCount=0;
    for (int y=h/3;y<h-2;++y) if (real(left[y],w,true)||real(right[y],w,false)) ++realCount;
    e.usable=realCount>=scaled(4,h,60);
    const auto lb=leftContour.empty()?breaks(left,w,true,s):contourBreaks(leftContour,w,h,s);
    e.leftBreaks=static_cast<int>(lb.size());
    if (lb.size()>=2) {
        e.down=lb[0].near; e.up=lb[1].near;
        e.mid=convexMid(left,w,e.down,e.up,s);
    } else if (lb.size()==1) {
        // A remaining opening gives the upper endpoint and its lower convex endpoint.
        // The FSM decides their meaning; do not rename the survivor as a new down corner.
        e.mid=lb[0].near;
        e.up=lb[0].far.valid()?lb[0].far:lb[0].near;
        const Point convex=convexMid(left,w,{left[h-3],h-3},e.mid,s);
        if(convex.valid()) e.mid=convex;
    }
    int good=0,jumps=0,total=0;
    const int y0=e.up.valid()?std::max(2,e.up.y-2):std::max(2,h/10);
    const int nearEnd=h-1-scaled(6,h,60);
    const int y1=e.down.valid()?std::min(nearEnd,e.down.y+2):nearEnd;
    const int jump=scaled(s.jumpPx80,w,80);
    for(int y=y0;y<=y1;++y) {
        ++total;
        if(real(right[y],w,false)) ++good;
        if(y>y0 && real(right[y],w,false) && real(right[y-1],w,false)) {
            const int delta=right[y]-right[y-1];
            if(std::abs(delta)>=jump) {
                // A steep but smooth right edge is still connected. Count an abrupt step,
                // not a sustained slope, by comparing the surrounding local growth.
                double expected=0;int neighbours=0;
                if(y>=2 && real(right[y-2],w,false)) {expected+=right[y-1]-right[y-2];++neighbours;}
                if(y+1<h && real(right[y+1],w,false)) {expected+=right[y+1]-right[y];++neighbours;}
                if(!neighbours || std::abs(delta-expected/neighbours)>=jump) ++jumps;
            }
        }
    }
    e.rightContinuous=total>0 && good*100>=total*s.rightContinuousPercent && jumps==0;

    // Untraced bumper rows are not genuine two-sided line loss.
    int missingRun=0;
    for(int y=h/3;y<=nearEnd;++y) {
        missingRun=!real(left[y],w,true)&&!real(right[y],w,false)?missingRun+1:0;
        e.bothMissingRows=std::max(e.bothMissingRows,missingRun);
    }
    for(int y=h-3;y>h/3+1;--y) {
        if(!real(right[y],w,false)) continue;
        if(real(right[y-2],w,false) && right[y-2]-right[y]>=jump &&
           real(right[y+1],w,false) && std::abs(right[y+1]-right[y])<jump) {
            e.rightOut={right[y],y};break;
        }
        // A true boundary can run out to x=w-1, which is not stored as a real edge.
        if(right[y-1]>=w-1-scaled(1,w,80) && right[y-1]<w &&
           right[y-1]-right[y]>=jump) { e.rightOut={right[y],y};break; }
        if(!real(right[y-1],w,false)) {
            int top=y-1;
            while(top>=h/3 && !real(right[top],w,false)) --top;
            if(top>=h/3 && right[top]-right[y]>=jump) { e.rightOut={right[y],y};break; }
        }
    }
    if(!rightContour.empty()) {
        std::vector<Point> mirrored;
        for(Point p:rightContour) mirrored.push_back({w-1-p.x,p.y});
        for(const auto& b:contourBreaks(mirrored,w,h,s)) {
            if(b.near.y>h/3+1 && (!e.rightOut.valid() || b.near.y>e.rightOut.y))
                e.rightOut={w-1-b.near.x,b.near.y};
        }
    }
    detectLowCameraEntry(e,left,right,w,s);
    return e;
}

void Controller::reset() { *this=Controller{}; }

Result Controller::update(const std::vector<int>& l, const std::vector<int>& r, int w,
                          long long now, bool allowStart, bool enabled, const Settings& s,
                          const std::vector<Point>& leftContour, const std::vector<Point>& rightContour,
                          bool enterNow, bool paused, bool templateHit) {
    Result out; out.left=l;out.right=r;out.center.assign(l.size(),-1);
    const int h=static_cast<int>(l.size());
    if(!enabled) { reset();return out; }
    auto freezeClock=[&]() {
        if(haveClock_) {
            const long long elapsed=std::max(0LL,now-lastNow_);
            stageAt_+=elapsed;
            if(cooldownUntil_>0) cooldownUntil_+=elapsed;
            if(creepAt_>0) creepAt_+=elapsed;   // 【停车左移】暂停时冻结左移计时
            if(forceLeftAt_>0) forceLeftAt_+=elapsed; // 【强制左转】暂停时冻结该窗口
        }
        lastNow_=now;haveClock_=true;wasPaused_=true;
        confirm_=absent_=lowConfirm_=0;
    };
    // Do this before size changes and detection: an empty stopped-camera frame
    // must not reset the FSM or count as a corner disappearing.
    if(paused) {
        freezeClock();out=lastResult_;out.changed=false;
        out.evidence={}; // Cached geometry is not a fresh camera observation.
        out.stage=stage_;out.completed=completed_;return out;
    }
    out.evidence=detect(l,r,w,s,leftContour,rightContour);
    if(!out.evidence.usable) {
        freezeClock();out.stage=stage_;out.completed=completed_;
        out.active=stage_>=1 && stage_<=5;
        // 【强制左转】几何不可用的帧不能把强制窗口打断
        if(s.forceLeftMs>0 && forceLeftAt_>0 && out.active && now-forceLeftAt_<s.forceLeftMs) {
            out.forceLeft=true;out.forceLeftError=s.forceLeftError;out.forceLeftSpeed=s.forceLeftSpeed;
        }
        return out;
    }
    if (w!=width_ || h!=height_) {
        reset();width_=w;height_=h;
        out.evidence=detect(l,r,w,s,leftContour,rightContour);
    }
    if(wasPaused_ && haveClock_) {
        const long long elapsed=std::max(0LL,now-lastNow_);
        stageAt_+=elapsed;
        if(cooldownUntil_>0) cooldownUntil_+=elapsed;
    }
    lastNow_=now;haveClock_=true;wasPaused_=false;
    const Evidence& e=out.evidence;
    auto transition=[&](int next) {
        if(next==0) {
            lowCameraMode_=false;lowConfirm_=0;lowEntryPosition_={};
            virtualDown_=lowMid_=lowUp_={};
        }
        // 【停车左移】离开阶段1就清掉左移子状态
        if(next!=1) { creepPhase_=0;creepTurns_=0.0;pendingTurns_=0.0; }
        // 【强制左转】每次真正进入入环阶段(2)都重开一次强制左打窗口
        if(next==2) forceLeftAt_=now;
        stage_=next;stageAt_=now;confirm_=absent_=0;out.changed=true;
    };
    if(h>=12 && r.size()==l.size() && w>=12) {
        if(enterNow && allowStart && !completed_ && stage_<=1 && e.usable) {
            entryAnchor_=lowCameraMode_ && lowUp_.valid()?lowUp_:e.up;
            if(e.lowCameraEntry) entryAnchor_=e.lowCameraUp;
            if(!entryAnchor_.valid()) {
                // 【修正】兜底锚点必须落在画面下半部。
                // 原实现从 y=h/3 向下找第一个 real(x>1) 的点：在"左边界贴边"的环岛
                // 入口画面里 (raw_left 整段为 0) 一个都找不到，entryAnchor_ 保持无效，
                // 于是 stage 2 的补线分支被整段跳过，中线退化成原始几何中心。
                // 现在先在下半部找真实边界，再退一步接受"贴边行"作为入口锚点。
                for(int y=h-3;y>=h/2;--y) if(real(l[y],w,true)) {entryAnchor_={l[y],y};break;}
                if(!entryAnchor_.valid())
                    for(int y=h-3;y>=h/2;--y) if(valid(l[y],w)) {entryAnchor_={l[y],y};break;}
            }
            transition(2);
        } else if(stage_==0) {
            // 【已删除·三点几何确认】原实现要求 down/mid/up 三个拐点同时有效且都在左半画面，
            // 实车环岛入口这几个点几乎恒为 -1，这条路从未成立过，已移除。
            const bool lowCandidate=allowStart && !completed_ && now>=cooldownUntil_ && e.lowCameraEntry;
            // 【修复】实车环岛入口常常只闪现 1~2 帧（车头扫过开口后左下角就不再贴边），
            // 原实现要求"连续 N 帧"且把下限硬编码成 3，实际永远凑不满，环岛路径直接死掉。
            // 现在：在 lowCameraWindowMs 时间窗口内累计候选帧，中间允许夹杂非候选帧；
            //       位置跳变不再清零，只刷新参考位置。
            if(lowCandidate) {
                // 窗口内：任何候选帧都计数并刷新参考位置；窗口过期或首次则重新从 1 开始。
                if(lowConfirm_==0 || lowEntryAt_==0 || now-lowEntryAt_>std::max(200,s.lowCameraWindowMs)) {
                    lowConfirm_=1;
                } else {
                    ++lowConfirm_;
                }
                lowEntryPosition_=e.rightEntry;
                lowEntryAt_=now;
            } else if(lowConfirm_>0 && lowEntryAt_>0 &&
                      now-lowEntryAt_>std::max(200,s.lowCameraWindowMs)) {
                lowConfirm_=0;lowEntryPosition_={};
            }
            // 【入环模板匹配】实车环岛入口左边界全程有效(detect 返回 leftBreaks=0, mid/up 全空)，
            // 相机根本拍不到环岛左侧拐点，几何检测这条路走不通。
            // 上层用 OpenCV 模板匹配给出 templateHit。
            //
            // 【不用 allowStart】allowStart 依赖 canSeek，而 canSeek 又被
            // marker_window_ms(14000)、黑块门控、人行横道/锥桶状态等一堆条件绑住 ——
            // 实测出现过"连续10帧命中(0.82)但因为 lap_ms=27460 超出14秒窗口而被挡死"。
            // 模板匹配是画面级识别，时间窗改由 NewTrack 侧用
            // [ringStartupBlockMs, ringTemplateMaxMs] 这个明确区间控制。
            if(s.templateEntryEnable && !completed_ && now>=cooldownUntil_ && templateHit) {
                lowCameraMode_=false;
                if(s.creepEnable) {
                    creepPhase_=1;creepAt_=now;creepTurns_=0.0;pendingTurns_=0.0;
                    transition(1);   // transition(1) 不会清 creep 子状态
                } else {
                    // 未启用停车左移时退化为直接切换
                    transition(2);
                }
            }
            else if(lowConfirm_>=std::max(1,s.lowCameraConfirmFrames)) {
                lowCameraMode_=true;virtualDown_=e.virtualDown;lowMid_=e.lowCameraMid;lowUp_=e.lowCameraUp;
                transition(1);
            }
        } else if(e.usable) {
            if(stage_==1) {
                if(lowCameraMode_ && e.lowCameraMid.valid() && e.lowCameraUp.valid()) {
                    if(e.virtualDown.valid()) virtualDown_=e.virtualDown;
                    lowMid_=e.lowCameraMid;lowUp_=e.lowCameraUp;
                }
                const Point curMid=lowCameraMode_?e.lowCameraMid:e.mid;
                const Point curUp =lowCameraMode_?(e.lowCameraUp.valid()?e.lowCameraUp:lowUp_):e.up;
                // 【入环停车左移】消耗上层喂入的车轮圈数（只在左移阶段累计）。
                const double dturns=pendingTurns_;pendingTurns_=0.0;
                if(creepPhase_>0) creepTurns_+=dturns;
                const double mmPerTurn=3.14159265358979323846*s.wheelDiameterMm;
                // 【修复】低相机模式下"左中拐点消失"的定义：
                //   曾经确认过中拐点(存有 lowMid_/lowUp_)，而现在这一帧已经不再显示它。
                // 原来的 !curMid.valid() && curUp.valid() 在低相机模式下几乎不可能成立，
                // 因为低相机的 mid/up 只在候选帧上出现，而候选帧在实车上只闪现 1~2 帧。
                const bool midGone=lowCameraMode_
                    ?(lowMid_.valid() && lowUp_.valid() && !e.lowCameraMid.valid())
                    :(!curMid.valid() && curUp.valid());
                if(!s.creepEnable) {
                    // 关闭该功能时保持原有的直接切换行为（含单断点的 onlyUp 快捷路径）。
                    const bool onlyUp=e.leftBreaks==1 && curUp.valid();
                    const bool advanced=curUp.valid() && curUp.y>=h*s.enterYPercent/100;
                    confirm_=(midGone||onlyUp||advanced)?confirm_+1:0;
                    if(confirm_>=std::max(1,s.confirmFrames)) { entryAnchor_=curUp;transition(2); }
                    else if(now-stageAt_>=s.approachTimeoutMs) transition(0);
                } else if(creepPhase_==0) {
                    // 检测到左中拐点消失 -> 进入停车
                    if(midGone) {
                        creepPhase_=1;creepAt_=now;creepTurns_=0.0;confirm_=0;out.changed=true;
                    } else if(now-stageAt_>=s.approachTimeoutMs) transition(0);
                } else if(creepPhase_==1) {
                    // 停车保持；若中拐点又出现则取消本次入环
                    if(curMid.valid()) { creepPhase_=0;creepTurns_=0.0; }
                    else if(now-creepAt_>=s.creepStopMs) { creepPhase_=2;creepAt_=now;creepTurns_=0.0; }
                } else if(creepPhase_==2) {
                    // 向左低速移动，边移动边找左上拐点
                    const bool found=curUp.valid();
                    const bool reached=creepTurns_*mmPerTurn>=s.creepMaxMm;
                    const bool timedOut=now-creepAt_>=s.creepTimeoutMs;
                    if(found) {
                        entryAnchor_=curUp;creepPhase_=0;transition(2);
                    } else if(reached||timedOut) {
                        // 走满 0.4m 仍未找到左上拐点：用当前几何兜底后进入补线
                        if(!entryAnchor_.valid()) {
                            for(int y=h-3;y>=h/2;--y) if(real(l[y],w,true)) {entryAnchor_={l[y],y};break;}
                            if(!entryAnchor_.valid())
                                for(int y=h-3;y>=h/2;--y) if(valid(l[y],w)) {entryAnchor_={l[y],y};break;}
                        }
                        creepPhase_=0;transition(2);
                    }
                }
                out.creepPhase=creepPhase_;
                out.creepTurns=creepTurns_;
                out.creepMm=creepTurns_*mmPerTurn;
                out.creepStop=(creepPhase_==1);
                out.creepMove=(creepPhase_==2);
            } else if(stage_==2) {
                const Point current=lowCameraMode_?e.lowCameraUp:e.up;
                if(current.valid()) {entryAnchor_=current;absent_=0;} else ++absent_;
                if(absent_>=s.disappearFrames || now-stageAt_>=s.entryTimeoutMs) transition(3);
            } else if(stage_==3) {
                // 【强制左转】强制窗口内不出环，避免"刚进环就被出环判据拉出去"
                const bool forceWindow=s.forceLeftMs>0 && forceLeftAt_>0 &&
                                       now-forceLeftAt_<s.forceLeftMs;
                const bool exit=!forceWindow && now-stageAt_>=s.exitSearchDelayMs && e.rightOut.valid() &&
                    e.bothMissingRows>scaled(s.bothMissingRows60,h,60);
                confirm_=exit?confirm_+1:0;
                if(confirm_>=s.confirmFrames) {exitAnchor_=e.rightOut;transition(4);}
            } else if(stage_==4) {
                if(e.rightOut.valid()) {exitAnchor_=e.rightOut;absent_=0;} else ++absent_;
                if(absent_>=s.disappearFrames) transition(5);
            } else if(stage_==5) {
                const bool pair=e.mid.valid() && e.up.valid() && e.mid.y>e.up.y;
                if(pair) recoveryPairSeen_=true;
                absent_=recoveryPairSeen_ && !e.up.valid()?absent_+1:0;
                if(absent_>=s.disappearFrames) {
                    completed_=true;cooldownUntil_=now+s.cooldownMs;transition(0);
                }
            }
        }
    }
    out.stage=stage_;out.completed=completed_;out.active=stage_>=1 && stage_<=5;
    // 【强制左转】进环后的一段固定左打窗口；上层据此覆盖 error 与速度。
    if(s.forceLeftMs>0 && forceLeftAt_>0 && out.active && now-forceLeftAt_<s.forceLeftMs) {
        out.forceLeft=true;
        out.forceLeftError=s.forceLeftError;
        out.forceLeftSpeed=s.forceLeftSpeed;
    }
    out.lowCameraEntryConfirmed=lowCameraMode_ && out.active;
    // The lower end of the surviving opening is a recovery anchor only in state 5.
    if(stage_>=1 && stage_<=2 && e.leftBreaks==1) out.evidence.mid={};
    if(!out.active || h<12 || r.size()!=l.size() || w<12) {lastResult_=out;return out;}

    if(stage_==1 && lowCameraMode_ && virtualDown_.valid() && lowMid_.valid() && lowUp_.valid()) {
        fitSegment(out.left,virtualDown_,lowMid_,w);
        fitSegment(out.left,lowMid_,lowUp_,w);
    } else if(stage_==1 && e.down.valid() && e.up.valid()) {
        if(e.mid.valid() && e.down.y>e.mid.y && e.mid.y>e.up.y) {
            fitSegment(out.left,e.down,e.mid,w);
            fitSegment(out.left,e.mid,e.up,w);
        } else fitSegment(out.left,e.down,e.up,w);
    } else if(stage_==2 && entryAnchor_.valid()) {
        const Point a=entryAnchor_;
        // 【入环补线】按新的入环定义：
        //   右边线 = 左上拐点 -> 画面右下角(w-1,h-1) 的连线；
        //            a.y 以上按同一直线外推（fitSegment 只覆盖 [a.y,h-1]）。
        //   左边线 = 画面最左侧，即全部行取 x=0。
        //   中线由下面的通用公式从左右边界得到。
        fitSegment(out.right,a,{w-1,h-1},w);
        if(a.y>0) {
            const double dy=double((h-1)-a.y);
            const double slope=dy>1e-9?double((w-1)-a.x)/dy:0.0;
            for(int y=0;y<a.y;++y)
                out.right[y]=clamp(static_cast<int>(std::lround(a.x+slope*(double(y)-a.y))),w);
        }
        std::fill(out.left.begin(),out.left.end(),0);
    } else if(stage_==3) {
        clearAbove(out.left,h/3);clearAbove(out.right,h/3);
    } else if(stage_==4 && exitAnchor_.valid()) {
        const Point target=exitTarget(r,exitAnchor_,w,s);
        clearAbove(out.right,exitAnchor_.y);
        fitSegment(out.right,target,exitAnchor_,w);
        clearAbove(out.left,h/3);clearAbove(out.right,h/3);
    } else if(stage_==5 && e.mid.valid() && e.up.valid() && e.mid.y>e.up.y) {
        fitSegment(out.left,e.up,e.mid,w);
    }
    // Border endpoints are useful geometry in the circle, but are not genuine detector evidence.
    //
    // 【修正】环岛入环(stage2)/环内(stage3)阶段把中线向内侧偏移。
    // 原实现所有阶段都取 (a+b)/2（几何正中）。实车数据证明这不够：
    //   stage 2/3 期间 steer_error 在 -131~+120 之间震荡，车直接冲过环岛。
    // 原因是环岛入口画面里左边界贴边(x=0)、右边界正常，"正中"仍然偏外。
    // 现在这两段按 s.ringCenterPratio（默认 30）取靠左的位置；
    // 其它阶段仍走 (a+b)/2，行为与原来完全一致。
    const bool ringInnerStage=(stage_==2||stage_==3);
    const int pratio=ringInnerStage?std::max(10,std::min(90,s.ringCenterPratio)):50;
    for(int y=0;y<h;++y) {
        const int a=out.left[y],b=out.right[y];
        if(valid(a,w)&&valid(b,w)&&b>a)
            out.center[y]=(pratio==50)?((a+b)/2):(a+(b-a)*pratio/100);
    }
    lastResult_=out;return out;
}

} }
