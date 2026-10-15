#include "NewTrack/GrayCone.hpp"
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iostream>
int main() {
    NewTrack::ConeSequence::Settings s;
    cv::Mat road(60,80,CV_8UC1,cv::Scalar(210));cv::rectangle(road,{0,0,7,60},{10},-1);
    auto cone=road.clone();cv::fillConvexPoly(cone,std::vector<cv::Point>{{57,17},{49,39},{64,39}},cv::Scalar(20));
    auto r=NewTrack::GrayCone::detect(cone,s);assert(r.observation.valid && r.observation.candidates.size()==1);
    assert(r.observation.candidates[0].acquire);cv::Mat color;cv::cvtColor(cone,color,cv::COLOR_GRAY2BGR);
    assert(NewTrack::GrayCone::detect(color,s).observation.candidates.size()==1);
    assert(NewTrack::GrayCone::detect(road,s).observation.candidates.empty());
    auto board=road.clone();cv::rectangle(board,{45,15,15,25},{20},-1);assert(NewTrack::GrayCone::detect(board,s).observation.candidates.empty());
    auto line=road.clone();cv::line(line,{35,10},{45,55},{20},3);assert(NewTrack::GrayCone::detect(line,s).observation.candidates.empty());
    auto clipped=road.clone();cv::fillConvexPoly(clipped,std::vector<cv::Point>{{57,35},{43,64},{70,64}},{20});
    for(const auto& candidate:NewTrack::GrayCone::detect(clipped,s).observation.candidates) assert(!candidate.acquire);
    assert(!NewTrack::GrayCone::detect(cv::Mat(60,80,CV_8UC1,cv::Scalar(0)),s).observation.valid);
    assert(!NewTrack::GrayCone::detect(cv::Mat(),s).observation.valid);
    cv::Mat bigger;cv::resize(cone,bigger,{160,120},0,0,cv::INTER_NEAREST);
    assert(NewTrack::GrayCone::detect(bigger,s).observation.candidates.size()==1);
    std::cout<<"PASS gray cone: gray/BGR equivalence, contrast/taper, board/line/edge rejection, invalid frames, scaling\n";
}
