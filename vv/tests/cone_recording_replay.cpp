// Diagnostic replay, not a proof of the vehicle's counterfactual trajectory.
#include "NewTrack/GrayCone.hpp"
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
int main(int argc,char** argv) {
    if(argc!=5) {std::cerr<<"Usage: replay INI manifest.csv image_directory output.csv\n";return 2;}
    NewTrack::ConeSequence::Settings settings;std::string error;
    if(!NewTrack::ConeSequence::loadFile(argv[1],settings,error)) {std::cerr<<error;return 2;}
    std::ifstream manifest(argv[2]);std::ofstream output(argv[4]);if(!manifest || !output) return 2;
    output<<"image,time_ms,candidates,acquirable,phase,entry_reason,visual_reason,near_seen,threshold\n";
    NewTrack::ConeSequence::Controller controller;std::string line;int frames=0,candidates=0,gaps=0;long long previous=-1;
    while(std::getline(manifest,line)) {
        const auto comma=line.find(',');if(comma==std::string::npos) return 2;
        const auto file=line.substr(0,comma);const long long ms=std::stoll(line.substr(comma+1));
        const auto image=cv::imread(std::string(argv[3])+"/"+file,cv::IMREAD_GRAYSCALE);if(image.empty()) return 2;
        auto result=NewTrack::GrayCone::detect(image,settings);int acquire=0;
        for(const auto& candidate:result.observation.candidates) acquire+=candidate.acquire;
        // Do not reconstruct road evidence or repeat saved images to fill missing captures.
        controller.update(ms,0,true,ms,result.observation.valid,settings,result.observation);
        output<<file<<','<<ms<<','<<result.observation.candidates.size()<<','<<acquire<<','<<NewTrack::ConeSequence::name(controller.phase())
            <<','<<controller.entryReason()<<','<<controller.reason()<<','<<controller.nearSeen()<<','<<result.threshold<<'\n';
        ++frames;candidates+=!result.observation.candidates.empty();if(previous>=0 && ms-previous>150) ++gaps;previous=ms;
    }
    std::cout<<"REPLAY frames="<<frames<<" candidate_frames="<<candidates<<" gaps_over_150ms="<<gaps<<" final="
        <<NewTrack::ConeSequence::name(controller.phase())<<"; sparse saved frames only; no vehicle success claim\n";
}
