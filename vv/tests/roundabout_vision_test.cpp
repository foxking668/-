#include "headfile.hpp"
#include "NewTrack/Roundabout.hpp"
#include <cassert>

using namespace NewTrack::Roundabout;

Result fromMask(const cv::Mat& mask,Controller* state=nullptr,long long now=0) {
    ImageProcess::FindLine finder;
    finder.submitCountorFrame(mask);
    finder.setMissingEdgeProxyEnabled(false);
    assert(finder.handleContoursInTopFromCountorFrameSimple());
    finder.findLineContoursSimple();
    std::vector<int> l(finder.getLeftLine(),finder.getLeftLine()+mask.rows);
    std::vector<int> r(finder.getRightLine(),finder.getRightLine()+mask.rows);
    std::vector<NewTrack::Roundabout::Point> contour;
    for(const auto p:finder.getLeftLineContours()) contour.push_back({p.x,p.y});
    std::vector<NewTrack::Roundabout::Point> rightContour;
    for(const auto p:finder.getRightLineContours()) rightContour.push_back({p.x,p.y});
    if(state) return state->update(l,r,mask.cols,now,true,true,Settings{},contour,rightContour);
    Result out;out.left=l;out.right=r;
    out.evidence=detect(l,r,mask.cols,Settings{},contour,rightContour);
    return out;
}

int main(int argc,char** argv) {
    cv::Mat mask(60,80,CV_8UC1,cv::Scalar(0));
    for(int y=0;y<60;++y) {
        int l=20,r=43+y/2;
        if(y<10 || (y>=29&&y<=43)) l=0;
        else if(y>=10&&y<=28) l=26-std::abs(y-18);
        for(int x=l;x<=r;++x) mask.at<unsigned char>(y,x)=255;
    }
    auto e=fromMask(mask).evidence;
    std::cout<<"Synthetic contour: D/M/U="<<e.down.y<<"/"<<e.mid.y<<"/"<<e.up.y
             <<" breaks="<<e.leftBreaks<<" right_ok="<<e.rightContinuous<<"\n";
    assert(e.down.valid()&&e.mid.valid()&&e.up.valid()&&e.rightContinuous);
    cv::Mat crossing=mask.clone();
    for(int y=29;y<=43;++y) for(int x=43+y/2;x<80;++x) crossing.at<unsigned char>(y,x)=255;
    e=fromMask(crossing).evidence;assert(!e.rightContinuous);
    cv::Mat straight(60,80,CV_8UC1,cv::Scalar(0));
    for(int y=0;y<60;++y) for(int x=20;x<=43+y/2;++x) straight.at<unsigned char>(y,x)=255;
    e=fromMask(straight).evidence;
    assert(e.leftBreaks==0&&!e.down.valid()&&!e.up.valid());
    // Complete FSM with binary masks through the actual contour extractor, including the exit.
    Controller fullState;long long clockMs=0;
    auto drive=[&](const cv::Mat& image,int delay=40) {
        clockMs+=delay;return fromMask(image,&fullState,clockMs);
    };
    drive(mask);assert(drive(mask).stage==1);
    cv::Mat entering=straight.clone();
    for(int y=0;y<60;++y) {
        entering.row(y).setTo(0);
        const int l=y<18?30:(y<=32?0:20);
        for(int x=l;x<=43+y/2;++x) entering.at<unsigned char>(y,x)=255;
    }
    drive(entering);assert(drive(entering).stage==2);
    drive(straight);drive(straight);assert(drive(straight).stage==3);
    cv::Mat exiting=straight.clone();
    for(int y=24;y<32;++y) exiting.row(y).setTo(255);
    for(int y=32;y<=36;++y) for(int x=43+y/2;x<=78;++x) exiting.at<unsigned char>(y,x)=255;
    drive(exiting,900);auto exited=drive(exiting);
    assert(exited.stage==4&&exited.evidence.bothMissingRows>5&&exited.evidence.rightOut.valid());
    drive(straight);drive(straight);assert(drive(straight).stage==5);
    const auto recovery=drive(entering);assert(recovery.evidence.mid.valid()&&recovery.evidence.up.valid());
    drive(straight);drive(straight);const auto done=drive(straight);
    assert(done.stage==0&&done.completed);
    std::cout<<"PASS full FSM through original contour extraction, including exit and recovery.\n";
    Controller state;
    Controller croppedState;
    cv::Mat preview(4*270,2*360,CV_8UC3,cv::Scalar(30,30,30));
    for(int i=1;i<argc;++i) {
        cv::Mat frame=cv::imread(argv[i]);assert(!frame.empty());
        // Optional supplied article figures: inspect only the upper-left original track image.
        if(frame.cols>1000 && frame.rows>800) {
            const cv::Rect rois[]={{28,34,477,358},{26,32,478,360},{28,33,477,360},{20,36,480,360}};
            frame=frame(rois[std::min(i-1,3)]).clone();
        }
        cv::resize(frame,frame,{80,60});
        ImageProcess::HandleImage hi;hi.submitOriginFrame(frame);
        const auto bin=hi.getGreyAndWhiteBinaryFromOriginalFrame();
        // Same pre-processing as NewTrack (11-pixel horizontal close).
        cv::Mat closed;
        cv::morphologyEx(bin,closed,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{11,1}));
        const auto closedEvidence=fromMask(closed).evidence;
        const auto raw=fromMask(bin);
        e=raw.evidence;
        std::cout<<"Reference "<<i<<": D/M/U="<<e.down.y<<"/"<<e.mid.y<<"/"<<e.up.y
                 <<" breaks="<<e.leftBreaks<<" right_ok="<<e.rightContinuous<<"\n";
        std::cout<<"  after close(11): breaks="<<closedEvidence.leftBreaks<<"\n";
        Result modified;
        if(i<=3) {
            for(int n=0;n<3;++n) modified=fromMask(bin,&state,i*200+n*40);
            assert(modified.stage==i); // Real figure sequence: approach, enter, inside.
            ImageProcess::HandleImage cropped;
            cropped.submitOriginFrame(frame(cv::Rect(0,3,80,56)).clone());
            Result croppedResult;
            for(int n=0;n<3;++n) croppedResult=fromMask(cropped.getGreyAndWhiteBinaryFromOriginalFrame(),&croppedState,i*200+n*40);
            assert(croppedResult.stage==i); // Same 5%/2% crop as the default car configuration.
        } else if(i==4) {
            assert(e.mid.valid()&&e.up.valid()&&e.mid.y>e.up.y);
            modified=raw;fitSegment(modified.left,e.up,e.mid,80);
            modified.stage=5;
        }
        if(i<=4) {
            cv::Mat overlay=frame.clone();
            for(int y=0;y<60;++y) {
                if(modified.left[y]>=0) overlay.at<cv::Vec3b>(y,modified.left[y])={0,255,0};
                if(modified.right[y]>=0) overlay.at<cv::Vec3b>(y,modified.right[y])={255,80,0};
                if(modified.left[y]>=0 && modified.right[y]>modified.left[y])
                    overlay.at<cv::Vec3b>(y,(modified.left[y]+modified.right[y])/2)={0,255,255};
            }
            for(const auto p:{e.down,e.mid,e.up}) if(p.valid()) cv::circle(overlay,{p.x,p.y},1,{255,0,255},-1);
            cv::Mat a,b;cv::resize(frame,a,{360,270},0,0,cv::INTER_NEAREST);
            cv::resize(overlay,b,{360,270},0,0,cv::INTER_NEAREST);
            cv::putText(b,"STATE "+std::to_string(modified.stage),{10,25},cv::FONT_HERSHEY_SIMPLEX,0.65,{0,255,255},2);
            a.copyTo(preview(cv::Rect(0,(i-1)*270,360,270)));
            b.copyTo(preview(cv::Rect(360,(i-1)*270,360,270)));
        }
    }
    const char* previewPath=std::getenv("ROUNDABOUT_PREVIEW_PATH");
    if(previewPath && argc>=5) cv::imwrite(previewPath,preview);
    std::cout<<"PASS original FindLine/FindStartPos/HSV integration (OpenCV).\n";
}
