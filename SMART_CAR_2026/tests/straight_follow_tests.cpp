#include "straight_follow.hpp"
#include <functional>
#include <iostream>
#include <limits>
using namespace car2026;
namespace {
int checks=0;
void check(bool condition,const char* message) {
    ++checks;if(!condition) throw std::runtime_error(message);
}
void throws(const std::function<void()>& operation,const char* message) {
    bool threw=false;try {operation();}catch(const std::exception&) {threw=true;}check(threw,message);
}
Observation line(double nearX=.5,double heading=0,int h=240) {
    Observation result;result.frameValid=true;result.blackPath.confidence=1;
    result.blackPath.x.resize(h);
    for(int y=0;y<h;++y) result.blackPath.x[y]=nearX+heading/(.45-.84)*(double(y)/h-.84);
    measurePath(result.blackPath);return result;
}
StraightFollowObservation confirm(StraightLineFollower& follower,const Observation& observation,double start=0) {
    StraightFollowObservation result;
    for(int n=0;n<3;++n) {
        result=follower.observe(observation,start+n*.2,start+n*.2);
        check(result.isStraight && result.hasErrors,"straight geometry and raw errors available before confirmation");
        check(result.hasSuggestion==(n==2),"only consecutive fresh stable lines produce a suggestion");
    }
    return result;
}
}
int main() {
    try {
        Params params;params.confirm_frames=3;
        StraightLineFollower offset(params,{.5,0});
        auto result=confirm(offset,line(.60));
        check(result.lateralError>.099 && result.suggestedCommand>0 && !result.aligned,
              "initial right offset is corrected rather than learned as the target");
        check(result.isStraight,"an offset line is still straight");
        StraightLineFollower left(params,{.5,0});
        result=confirm(left,line(.40));
        check(result.suggestedCommand<0,"left offset has the opposite forward correction");
        StraightLineFollower direction(params,{.5,0});
        result=confirm(direction,line(.5,.04));
        check(result.isStraight && std::abs(result.lateralError)<1e-12 &&
              result.headingFeatureError>.039 && result.suggestedCommand>0 && !result.aligned,
              "oblique straight geometry is accepted but its direction error is corrected");
        StraightLineFollower calibrated(params,{.57,.02});
        result=confirm(calibrated,line(.57,.02));
        check(result.aligned && std::abs(result.suggestedCommand)<1e-12,
              "explicit camera target need not be centered or vertical");
        StraightLineFollower centred(params,{.5,0});
        result=confirm(centred,line(.5));
        check(result.aligned && result.suggestedCommand==0,"aligned target produces no corrective bias");
        result=centred.observe(line(.503),.6,.6);
        check(result.suggestedCommand==0,"pixel-scale deadband avoids small corrective jitter");
        auto curve=line();
        for(int y=0;y<240;++y) curve.blackPath.x[y]=.5+.6*std::pow(double(y)/240-.65,2);
        measurePath(curve.blackPath);
        result=centred.observe(curve,.8,.8);
        check(result.state=="NOT_STRAIGHT" && !result.isStraight && !result.hasSuggestion,
              "stable curved paths never enter straight following");
        result=confirm(centred,line(.50),1.);
        check(result.aligned && result.suggestedCommand==0,"curve recovery keeps the explicit target");

        auto partial=line();for(int y=95;y<120;++y) partial.blackPath.x[y]=-1;
        StraightLineFollower quality(params,{.5,0});confirm(quality,line());
        result=quality.observe(partial,.6,.6);
        check(result.state=="PARTIAL_PATH" && !result.hasErrors,"missing far support cannot use a heading fallback");
        auto bad=line();bad.blackPath.ambiguous=true;
        result=quality.observe(bad,.8,.8);
        check(result.state=="AMBIGUOUS" && !result.hasSuggestion,"branch ambiguity cancels advice immediately");
        bad=line();bad.blackPath.discontinuous=true;
        result=quality.observe(bad,1.,1.);
        check(result.state=="DISCONTINUOUS","mixed fragments cannot create advice");
        bad=line();bad.blackPath.confidence=.64;
        result=quality.observe(bad,1.2,1.2);
        check(result.state=="LOW_QUALITY","weak path coverage is not accepted");
        bad=line();bad.frameValid=false;
        result=quality.observe(bad,1.4,1.4);
        check(result.state=="INVALID_FRAME","invalid images withdraw advice");
        result=quality.observe(line(.65),1.6,1.6);
        check(result.state=="FEATURE_JUMP" && !result.hasSuggestion,
              "lost-path recovery cannot silently switch to a distant straight branch");
        result=confirm(quality,line(),1.8);
        check(result.aligned,"nearby original line can recover after fresh confirmation");
        result=quality.observe(line(.5,0,120),2.4,2.4);
        check(result.state=="PATH_SIZE_CHANGED","processing size changes require explicit new-session reset");
        quality.reset();result=confirm(quality,line(.6,0,120));
        check(result.isStraight && result.lateralError>.09,"explicit reset preserves target while allowing a new path session");

        StraightLineFollower clock(params,{.5,0});confirm(clock,line(.6));
        result=clock.observe(line(.6),.4,.5);
        check(result.state=="REPEATED_FRAME" && !result.hasSuggestion,"duplicate frames cannot accumulate output");
        result=clock.observe(line(.6),.3,.5);
        check(result.state=="BACKWARD_TIME","frame timestamp reversal is rejected");
        result=clock.observe(line(.6),.6,1.1);
        check(result.state=="STALE_FRAME","delayed processing withdraws advice");
        result=clock.observe(line(.6),1.3,1.3);
        check(result.state=="FRAME_GAP","long camera gaps force reconfirmation");
        result=confirm(clock,line(.6),1.5);
        check(result.lateralError>.09 && result.suggestedCommand<=.600001,
              "recovery starts output from zero and does not learn displaced target");
        result=clock.observe(line(),2.1,2.0);
        check(result.state=="INVALID_TIME","processing cannot precede capture");
        result=clock.observe(line(),2.1,1.8);
        check(result.state=="INVALID_TIME","processing clock cannot go backwards");
        result=clock.observe(line(),std::numeric_limits<double>::quiet_NaN(),2.2);
        check(result.state=="INVALID_TIME" && std::isfinite(result.suggestedCommand),"NaN cannot leak into advice");

        StraightImageFeatures features;
        auto noise=line();noise.blackPath.x[151]+=.009;
        check(fitStraightImageFeatures(noise.blackPath,features) && features.excludedRows==1,
              "one bounded isolated pixel displacement remains a supported straight");
        noise.blackPath.x[151]+=.01;
        check(!fitStraightImageFeatures(noise.blackPath,features),"large outlier cannot be hidden by fitting");
        auto malformed=line();malformed.blackPath.x[160]=std::numeric_limits<double>::infinity();
        check(!fitStraightImageFeatures(malformed.blackPath,features),"nonfinite points are rejected");
        malformed=line();malformed.blackPath.x[160]=1.1;
        check(!fitStraightImageFeatures(malformed.blackPath,features),"out-of-range path points are rejected");
        malformed=line(.5,0,23);
        check(!fitStraightImageFeatures(malformed.blackPath,features),"short fit input is bounds-safe");
        malformed=line();for(int y=100;y<145;++y) malformed.blackPath.x[y]=-1;
        check(!fitStraightImageFeatures(malformed.blackPath,features),"broad missing support cannot masquerade as a straight");

        Params bounded=params;bounded.straight_lateral_gain=100;bounded.straight_heading_gain=0;
        bounded.straight_filter_s=0;
        StraightLineFollower limited(bounded,{.5,0});
        double previous=0;
        for(int n=0;n<20;++n) {
            result=limited.observe(line(.7),n*.2,n*.2);
            if(!result.hasSuggestion) continue;
            check(std::abs(result.suggestedCommand)<=5 &&
                  std::abs(result.suggestedCommand-previous)<=.600001,"command amplitude and elapsed-time slew remain bounded");
            previous=result.suggestedCommand;
        }
        check(previous==5,"large fixed error saturates at the diagnostic command limit");
        auto branch=line(.85);branch.blackPath.ambiguous=true;
        result=limited.observe(branch,4.,4.);
        check(!result.hasSuggestion && result.suggestedCommand==0,"invalid advice never exposes retained saturated output");

        Params once=params;once.confirm_frames=1;once.straight_filter_s=0;
        StraightLineFollower first(once,{.5,0});result=first.observe(line(.6),0,0);
        check(result.hasSuggestion && result.suggestedCommand==0,"first timestamp has no elapsed time for a turn step");
        result=first.observe(line(.6),.2,.2);
        check(std::isfinite(result.suggestedCommand) && result.suggestedCommand>0,"zero filter duration works on subsequent fresh frames");
        throws([&]{StraightLineFollower invalid(params,{.5,std::numeric_limits<double>::quiet_NaN()});},"invalid target fails startup");
        throws([&]{StraightLineFollower invalid(params,{1.1,0});},"target outside image fails startup");
        throws([&]{StraightLineFollower invalid(params,{.1,-.2});},"target leaving the fitted interval fails startup");
        for(double Params::* field : {&Params::straight_lateral_gain,&Params::straight_heading_gain,
             &Params::straight_filter_s,&Params::straight_deadband_image,
             &Params::straight_max_command,&Params::straight_command_rate_s}) {
            auto invalid=params;invalid.*field=std::numeric_limits<double>::quiet_NaN();
            throws([&]{StraightLineFollower controller(invalid,{.5,0});},"nonfinite tuning fails startup");
        }
        std::cout<<"PASS "<<checks<<" straight-follow checks; forward image advice only\n";return 0;
    }catch(const std::exception& error) {std::cerr<<"FAIL after "<<checks<<" checks: "<<error.what()<<'\n';return 1;}
}
