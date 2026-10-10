// Ordinary-lane helpers from cc(1) NewTrack.cpp; see ORIGIN.md.
struct Config {
    int centerBlackEnable=1,centerBlackMinRun=2,centerBlackMaxRun=18,centerBlackMaxStep=24;
    int centerBlackMinValidRows=8,centerBlackMinNearRows=2,centerBlackMinWhiteMargin=2;
    int singleEdgeEnable=1,singleEdgeBiasPx160=3;
    int partialBlackAssistEnable=1,partialBlackMinValidRows=3,partialBlackMinNearRows=1,partialBlackBlendPercent=70;
};
    static inline int clampi(int v, int lo, int hi)
    {
        return std::max(lo, std::min(v, hi));
    }
    static cv::Mat cropSafe(const cv::Mat &input, double top, double bottom, double left, double right)
    {
        if (input.empty()) return {};
        const int y0 = clampi(static_cast<int>(input.rows * top), 0, std::max(0,input.rows-1));
        const int y1cut = clampi(static_cast<int>(input.rows * bottom), 0, std::max(0,input.rows-1));
        const int x0 = clampi(static_cast<int>(input.cols * left), 0, std::max(0,input.cols-1));
        const int x1cut = clampi(static_cast<int>(input.cols * right), 0, std::max(0,input.cols-1));
        const int w = input.cols - x0 - x1cut;
        const int h = input.rows - y0 - y1cut;
        if (w < 8 || h < 8) return input.clone();
        return input(cv::Rect(x0,y0,w,h)).clone();
    }

    using cc_detail::centerToError;

    static bool averageCenter(const std::vector<int> &center, double &out)
    {
        if (center.empty()) return false;
        const int h = static_cast<int>(center.size());
        const int y0 = h/4, y1 = h*3/4;
        long long sum=0; int n=0;
        for(int y=y0;y<=y1 && y<h;++y) if(center[y]>=0){sum+=center[y];++n;}
        if(!n) return false;
        out=static_cast<double>(sum)/n;
        return true;
    }

    struct LaneLines
    {
        bool valid=false;
        std::vector<int> left, center, right;
        double aggregateCenter=0.0;
        int bilateralRows=0;
        int leftOnlyRows=0;
        int rightOnlyRows=0;
        bool singleEdgeUsed=false;
    };

    // 保存“正常道路”的逐行宽度记忆。这样转弯时一侧边线完全出画，
    // 可以用另一侧真实边线 + 历史道路宽度恢复虚拟边线，而不是直接把缺边补到0/图像最右端。
    static std::vector<double> laneWidthProfile;
    static double laneWidthMedian = 0.0;
    static int laneWidthProfileHeight = 0;

    static double medianOf(std::vector<int> values)
    {
        if(values.empty()) return 0.0;
        const auto mid=values.begin()+values.size()/2;
        std::nth_element(values.begin(),mid,values.end());
        return *mid;
    }

    static void repairSingleEdge(LaneLines &lane, int w, int h, const Config &c)
    {
        if((int)laneWidthProfile.size()!=h)
        {
            laneWidthProfile.assign(h,0.0);
            laneWidthProfileHeight=h;
            laneWidthMedian=0.0;
        }

        std::vector<int> currentWidths;
        for(int y=h/4;y<h*9/10;++y)
        {
            const int l=lane.left[y], r=lane.right[y];
            if(l>=0 && r>l+8)
            {
                const int width=r-l;
                // 已有宽度记忆后，明显突然变宽（锥桶宽道/环岛开口）不用于刷新正常路宽记忆。
                bool sane = laneWidthMedian<=0.0 ||
                            (width>=laneWidthMedian*0.72 && width<=laneWidthMedian*1.28);
                if(sane)
                {
                    currentWidths.push_back(width);
                    if(laneWidthProfile[y]<=0.0) laneWidthProfile[y]=width;
                    else if(width>=laneWidthProfile[y]*0.68 && width<=laneWidthProfile[y]*1.32)
                        laneWidthProfile[y]=laneWidthProfile[y]*0.82+width*0.18;
                }
            }
        }
        if((int)currentWidths.size()>=4)
        {
            const double m=medianOf(currentWidths);
            laneWidthMedian = laneWidthMedian<=0.0 ? m : laneWidthMedian*0.88+m*0.12;
        }

        if(!c.singleEdgeEnable) return;

        auto widthAt=[&](int y)->double
        {
            if(y>=0 && y<h && laneWidthProfile[y]>8.0) return laneWidthProfile[y];
            for(int d=1;d<=8;++d)
            {
                if(y-d>=0 && laneWidthProfile[y-d]>8.0) return laneWidthProfile[y-d];
                if(y+d<h && laneWidthProfile[y+d]>8.0) return laneWidthProfile[y+d];
            }
            return laneWidthMedian;
        };

        for(int y=0;y<h;++y)
        {
            int &l=lane.left[y], &r=lane.right[y], &m=lane.center[y];
            if(l>=0 && r>l)
            {
                ++lane.bilateralRows;
                m=(l+r)/2;
                continue;
            }
            const double roadWidth=widthAt(y);
            if(roadWidth<=8.0) { m=-1; continue; }
            if(l<0 && r>=0)
            {
                l=clampi((int)std::lround(r-roadWidth),0,std::max(0,r-2));
                m=(l+r)/2;
                ++lane.rightOnlyRows;
                lane.singleEdgeUsed=true;
            }
            else if(r<0 && l>=0)
            {
                r=clampi((int)std::lround(l+roadWidth),std::min(w-1,l+2),w-1);
                m=(l+r)/2;
                ++lane.leftOnlyRows;
                lane.singleEdgeUsed=true;
            }
            else m=-1;
        }
    }

    static LaneLines extractLane(const cv::Mat &mask, const Config &c)
    {
        LaneLines out;
        out.left.assign(mask.rows,-1); out.center.assign(mask.rows,-1); out.right.assign(mask.rows,-1);
        // 725分支直接复用你最后给的“7.25自动排期”里的 FindLine + FixLine 核心。
        // 只把原来的图像边缘代理关闭，改由新赛道的历史路宽单边界补线接手。
        FindLine findLine;
        findLine.submitCountorFrame(mask);
        findLine.setMissingEdgeProxyEnabled(false);
        if(!findLine.handleContoursInTopFromCountorFrameSimple()) return out;
        findLine.findLineContoursSimple();
        int *l=findLine.getLeftLine(); int *r=findLine.getRightLine(); int *m=findLine.getCenterLine();
        if(!l || !r || !m) return out;
        FixLine fix;
        fix.submitLeftStartPos(findLine.getLeftStartPos());
        fix.submitRightStartPos(findLine.getRightStartPos());
        fix.submitLeftEndPos(findLine.getLeftEndPos());
        fix.submitRightEndPos(findLine.getRightEndPos());
        fix.submitLeftLineContours(findLine.getLeftLineContours());
        fix.submitRightLineContours(findLine.getRightLineContours());
        fix.image_width=mask.cols; fix.image_height=mask.rows;
        fix.submitLeftLine(l); fix.submitRightLine(r); fix.submitCenterLine(m);
        fix.setMissingEdgeProxyEnabled(false);
        fix.fixLineSimple();
        for(int y=0;y<mask.rows;++y){out.left[y]=l[y];out.right[y]=r[y];out.center[y]=m[y];}
        repairSingleEdge(out,mask.cols,mask.rows,c);
        if(!averageCenter(out.center,out.aggregateCenter)) return out;
        out.valid=true;
        return out;
    }

    struct BlackLineResult
    {
        bool reliable=false;
        int validRows=0;
        int nearRows=0;
        double weightedX=0.0;
        std::vector<int> x;
    };

    static BlackLineResult detectCenterBlack(const cv::Mat &rawMask, const LaneLines &lane, const Config &c)
    {
        BlackLineResult out; out.x.assign(rawMask.rows,-1);
        if(!c.centerBlackEnable || !lane.valid) return out;
        int prediction=-1, predY=rawMask.rows-1;
        long long sum=0, weight=0;
        const int yNear=rawMask.rows*2/3;
        for(int y=rawMask.rows-3;y>=std::max(4,rawMask.rows/7);y-=2)
        {
            int l=lane.left[y], r=lane.right[y];
            if(l<0||r<0||r-l<18) continue;
            l=clampi(l+2,1,rawMask.cols-3); r=clampi(r-2,2,rawMask.cols-2);
            int reference=prediction>=0?prediction:((lane.center[y]>=0)?lane.center[y]:(l+r)/2);
            int maxStep=prediction>=0?c.centerBlackMaxStep*std::max(1,(predY-y)/2):rawMask.cols;
            int best=-1,bestScore=1<<30;
            const uchar *row=rawMask.ptr<uchar>(y);
            for(int x=l;x<=r;)
            {
                if(row[x]!=0){++x;continue;}
                int s=x; while(x<=r && row[x]==0) ++x; int e=x-1;
                int span=e-s+1;
                if(span<c.centerBlackMinRun || span>c.centerBlackMaxRun) continue;
                int m=std::max(c.centerBlackMinWhiteMargin,span/2);
                if(s-m<l || e+m>r) continue;
                if(row[s-m]==0 || row[e+m]==0) continue;
                int mid=(s+e)/2, d=std::abs(mid-reference);
                if(d>maxStep) continue;
                int score=d*10+span;
                if(score<bestScore){bestScore=score;best=mid;}
            }
            if(best>=0)
            {
                out.x[y]=best; prediction=best; predY=y; ++out.validRows;
                if(y>=yNear) ++out.nearRows;
                int w=(y>=yNear)?3:(y>=rawMask.rows/2?2:1); sum+=best*w; weight+=w;
            }
        }
        out.reliable=out.validRows>=c.centerBlackMinValidRows && out.nearRows>=c.centerBlackMinNearRows;
        if(weight) out.weightedX=(double)sum/weight;
        return out;
    }

