#include "core.hpp"
#include "imu.hpp"
#include <iostream>
#include <functional>
using namespace car2026;
namespace {
int checks=0;
void check(bool value,const char* message) {++checks;if(!value) throw std::runtime_error(message);}
void rejects(const std::function<void()>& f,const char* message) {
    bool threw=false;try {f();}catch(const std::exception&) {threw=true;}check(threw,message);
}
void rectangle(Image& i,int a,int b,int c,int d,Pixel p) {
    for(int y=b;y<d;++y) for(int x=a;x<c;++x) i.at(x,y)=p;
}
Image road() {Image i(320,240,{0,90,210});rectangle(i,48,0,272,240,{240,240,240});return i;}
Image guideLine(int x=156) {
    Image image=road();rectangle(image,x,20,x+8,240,{0,0,0});return image;
}
Path good() {Path p;p.x.assign(240,.5);p.confidence=1;return p;}
void checkGuideContrastAndIdentity(const Params& params) {
    // The dim floor is below black_v_max, just like the real front-camera clips.
    Image dim(320,240,{72,72,72});rectangle(dim,172,20,192,240,{18,18,18});
    Vision contrastVision(params);
    auto observation=contrastVision.analyze(dim,Stage::GarageReverse);
    check(observation.blackPath.confidence>.9,"dim floor does not merge with darker guide tape");
    check(std::abs(observation.blackPath.x[200]-181.5/320)<.01,"dim guide center is retained");
    Image weak(320,240,{72,72,72});rectangle(weak,172,20,192,240,{65,65,65});
    observation=contrastVision.analyze(weak,Stage::GarageReverse);
    check(observation.blackPath.confidence==0,"weak shading is not a confident dark guide");

    Vision forkVision(params);
    Image fork=road();rectangle(fork,144,20,152,240,{0,0,0});
    rectangle(fork,169,20,177,240,{0,0,0});
    observation=forkVision.analyze(fork,Stage::GarageReverse);
    check(observation.blackPath.ambiguous && observation.blackPath.confidence==0,
          "equally plausible nearby branches are explicitly uncertain");

    Vision parkingVision(params);
    parkingVision.analyze(guideLine(116),Stage::GarageReverse);
    parkingVision.analyze(road(),Stage::GarageReverse);
    for(int frame=0;frame<int(params.confirm_frames)+1;++frame) {
        observation=parkingVision.analyze(guideLine(236),Stage::GarageReverse);
        check(observation.blackPath.confidence==0,"parking cannot silently replace a lost reference");
    }
    observation=parkingVision.analyze(guideLine(120),Stage::GarageReverse);
    check(observation.blackPath.confidence>.9,"same nearby parking reference may recover locally");
    observation=parkingVision.analyze(Image(320,240),Stage::GarageReverse);
    check(!observation.frameValid,"camera blackout is explicitly invalid during parking");
    parkingVision.analyze(Image{},Stage::GarageReverse);
    for(int frame=0;frame<int(params.confirm_frames)+1;++frame) {
        observation=parkingVision.analyze(guideLine(236),Stage::GarageReverse);
        check(observation.blackPath.confidence==0,"invalid camera frames do not erase parking identity");
    }
    observation=parkingVision.analyze(guideLine(120),Stage::GarageReverse);
    check(observation.blackPath.confidence>.9,"same reference recovers after camera blackout");
    parkingVision.resetTracking();
    for(int frame=0;frame<int(params.confirm_frames);++frame)
        observation=parkingVision.analyze(guideLine(236),Stage::GarageReverse);
    check(observation.blackPath.confidence>.9,"explicit reset permits a new parking reference");

    Vision clippedVision(params);
    Image clipped=guideLine(200);rectangle(clipped,48,220,272,240,{240,240,240});
    observation=clippedVision.analyze(clipped,Stage::GarageReverse);
    check(observation.blackPath.confidence>.8 && observation.blackPath.x[228]<0,
          "reference can be observed above missing bottom rows");
    clipped=guideLine(212);rectangle(clipped,48,220,272,240,{240,240,240});
    observation=clippedVision.analyze(clipped,Stage::GarageReverse);
    check(observation.blackPath.confidence>.8 && observation.blackPath.x[200]>.66,
          "missing bottom rows preserve the prior nearby reference instead of center reset");

    Vision jumpVision(params);
    Image jump=guideLine(156);
    rectangle(jump,48,20,272,160,{240,240,240});
    rectangle(jump,180,20,188,160,{0,0,0});
    observation=jumpVision.analyze(jump,Stage::GarageReverse);
    check(observation.blackPath.discontinuous && observation.blackPath.confidence==0,
          "parking rejects high-coverage fragments with an abrupt cross-row jump");
    Vision obliqueVision(params);
    Image oblique=road();
    for(int y=20;y<240;++y) {
        const int x=100+y/4;
        rectangle(oblique,x,y,x+8,y+1,{0,0,0});
    }
    observation=obliqueVision.analyze(oblique,Stage::GarageReverse);
    check(observation.blackPath.confidence>.9 && !observation.blackPath.discontinuous,
          "gradual oblique guide remains usable after continuity checks");

    Vision perspectiveVision(params);
    Image perspective=road();
    for(int y=20;y<240;++y) {
        const int width=12+y/8;
        rectangle(perspective,160-width/2,y,160+(width+1)/2,y+1,{0,0,0});
    }
    observation=perspectiveVision.analyze(perspective,Stage::GarageReverse);
    check(observation.blackPath.confidence>.9,"parking accepts gradually wider near-field tape");
    Image broad=road();rectangle(broad,120,20,200,240,{0,0,0});
    observation=perspectiveVision.analyze(broad,Stage::GarageReverse);
    check(observation.blackPath.confidence==0,"perspective allowance does not accept a broad dark region");
}
void checkBlackReacquisition(const Params& params) {
    Vision vision(params);
    auto observation=vision.analyze(guideLine(116),Stage::ToCross);
    check(observation.blackPath.confidence>.9,"left guide line initially tracked");
    observation=vision.analyze(road(),Stage::ToCross);
    check(observation.blackPath.confidence==0,"missing line is not fabricated");
    for(int frame=1;frame<=int(params.confirm_frames);++frame) {
        observation=vision.analyze(guideLine(236),Stage::ToCross);
        if(frame<int(params.confirm_frames))
            check(observation.blackPath.confidence<params.line_min_confidence,"reacquisition requires stable frames");
    }
    check(observation.blackPath.confidence>.9 && observation.blackPath.lateral>.2,
          "shifted line outside old and central search windows is reacquired");
    observation=vision.analyze(guideLine(240),Stage::ToCross);
    check(observation.blackPath.confidence>.9,"reacquired line resumes local tracking");

    vision.resetTracking();
    vision.analyze(guideLine(116),Stage::ToCross);
    Image fork=guideLine(52);rectangle(fork,252,20,260,240,{0,0,0});
    for(int frame=0;frame<int(params.confirm_frames)+1;++frame) {
        observation=vision.analyze(fork,Stage::ToCross);
        check(observation.blackPath.confidence<params.line_min_confidence,"ambiguous distant branches are not reacquired");
    }
    vision.analyze(guideLine(236),Stage::ToCross);
    vision.analyze(road(),Stage::ToCross);
    for(int frame=1;frame<=int(params.confirm_frames);++frame) {
        observation=vision.analyze(guideLine(236),Stage::ToCross);
        if(frame<int(params.confirm_frames))
            check(observation.blackPath.confidence<params.line_min_confidence,"missing frame resets reacquisition confirmation");
    }
    check(observation.blackPath.confidence>.9,"line can be reacquired after interrupted confirmation");

    vision.resetTracking();
    vision.analyze(guideLine(116),Stage::ToCross);
    Image boundary=road();rectangle(boundary,264,20,272,240,{0,0,0});
    for(int frame=0;frame<int(params.confirm_frames)+1;++frame) {
        observation=vision.analyze(boundary,Stage::ToCross);
        check(observation.blackPath.confidence<params.line_min_confidence,"reacquisition rejects black at road boundary");
    }

    Vision recoveryVision(params);Mission recoveryMission(params);Telemetry telemetry;
    telemetry.yawValid=telemetry.yawMeasured=true;
    auto update=[&](const Image& image) {
        const auto detected=recoveryVision.analyze(image,recoveryMission.stage());
        return recoveryMission.update(detected,telemetry,recoveryVision.avoidCones(detected,image));
    };
    update(guideLine(116));
    telemetry.time+=.05;
    auto command=update(road());
    check(command.stage!=Stage::Fault,"short gap holds mission path during reacquisition");
    for(int frame=0;frame<int(params.confirm_frames);++frame) {
        telemetry.time+=.05;command=update(guideLine(236));
        check(command.stage!=Stage::Fault,"confirmed reacquisition completes within path hold window");
    }
    check(command.steer>0,"mission follows reacquired right-side guide line");
    telemetry.time+=1;command=update(road());
    check(command.stage==Stage::Fault && command.speed==0,"long gap still locks mission fault");
    telemetry.time+=.05;command=update(guideLine(236));
    check(command.stage==Stage::Fault && command.speed==0,"recovered perception cannot restart faulted mission");
}
void checkConeRingTransition(const Params& params) {
    Vision vision(params);Mission mission(params);Telemetry telemetry;
    telemetry.yawValid=telemetry.yawMeasured=true;telemetry.speed=.1;
    auto step=[&](const Image& image,double distance) {
        telemetry.time+=.1;telemetry.distance+=distance;
        const auto observation=vision.analyze(image,mission.stage());
        const auto avoidance=vision.avoidCones(observation,image);
        const auto command=mission.update(observation,telemetry,avoidance);
        check(command.stage!=Stage::Fault,"image-driven cone/ring transition must not fault");
    };
    step(guideLine(),0);step(guideLine(),params.garage_depart_m+.01);
    Image cone=guideLine();rectangle(cone,118,125,134,160,{0,90,210});
    for(int frame=0;frame<int(params.confirm_frames);++frame) step(cone,.02);
    check(mission.stage()==Stage::Cones,"image-driven cones entry");
    Image ring=guideLine();rectangle(ring,130,100,190,154,{0,90,210});
    for(int frame=0;frame<int(params.clear_frames);++frame)
        step(ring,params.cone_min_travel_m/params.clear_frames+.01);
    check(mission.stage()==Stage::ToRing,"visible ring island does not keep cones stage active");
    for(int frame=0;frame<int(params.confirm_frames);++frame) step(ring,.01);
    check(mission.stage()==Stage::RingEntry,"image-driven roundabout entry after cones");
}
struct Simulation {
    Params p;Mission mission;Observation o;Telemetry t;Command command;
    explicit Simulation(Params parameters):p(parameters),mission(p) {
        o.frameValid=true;o.blackPath=o.roadPath=good();o.barObservable=true;
        t.yawValid=t.yawMeasured=true;
    }
    void step(double distance=0,double yaw=0,double speed=.1,double dt=.1) {
        t.time+=dt;t.distance+=distance;t.yaw+=yaw;t.speed=speed;
        command=mission.update(o,t,o.roadPath);
        if(command.stage==Stage::Fault) throw std::runtime_error(command.reason);
    }
    void repeat(int n,double distance=0) {for(int i=0;i<n;++i) step(distance);}
    void reachRing() {
        o.cones={{100,100,10,15,150,.33,.5}};repeat(3);check(mission.stage()==Stage::Cones,"cones entry");
        o.cones.clear();repeat(5,.11);check(mission.stage()==Stage::ToRing,"cones exit");
        o.ringCandidate=true;repeat(3);o.ringCandidate=false;check(mission.stage()==Stage::RingEntry,"ring entry");
        step(.26);check(mission.stage()==Stage::RingLap,"ring lap");
    }
    void finishRing() {
        // Black exit is visible at the entrance too: it must NOT trigger an early exit.
        repeat(4,.05);check(mission.stage()==Stage::RingLap,"premature ring exit");
        for(int i=0;i<18;++i) step(.05,-p.ring_side*20);
        repeat(3);check(mission.stage()==Stage::RingExit,"full ring exit");
        repeat(5,.06);check(mission.stage()==Stage::ToCross,"ring exit confirm");
    }
    void crossing(bool bar) {
        o.stripe=true;o.stripeY=.82;o.bar=bar;repeat(3);check(mission.stage()==Stage::CrossApproach,"cross approach");
        step();
        if(bar) {
            check(mission.stage()==Stage::CrossWait,"bar stop");step(0,0,.04,1);
            o.bar=false;repeat(5);check(mission.stage()==Stage::CrossWait,"must wait three seconds stationary");
            step(0,0,0);check(command.speak,"speak on actual stop");
            for(int i=0;i<35;++i) step(0,0,0);
            check(mission.stage()==Stage::CrossPass,"bar release");
        } else check(mission.stage()==Stage::CrossPass,"clear crossing");
        o.stripe=false;repeat(8,.08);check(mission.stage()==Stage::ToGarage,"cross exit");
        o.garage=true;repeat(5,.04);o.garage=false;
    }
};
}
int main(int argc,char** argv) {
    try {
        Params p=argc>1?Params::load(argv[1]):Params{};p.validate();
        check(p.motion_calibrated==0,"shipping config must not drive by default");
        Params bad=p;bad.ring_side=0;rejects([&]{bad.validate();},"invalid turn sign");
        bad=p;bad.confirm_frames=2.5;rejects([&]{bad.validate();},"fractional frame count");
        bad=p;bad.cross_stop_s=2;rejects([&]{bad.validate();},"stop below rule minimum");
        YawTracker yaw;check(yaw.ingest(179,0),"yaw initialization");check(yaw.ingest(-179,.1),"yaw wrap");
        check(std::abs(yaw.value()-2)<1e-8,"yaw wrap adds 2 not 358");
        check(!yaw.ingest(-160,1),"yaw gap rejected");check(!yaw.fresh(1),"yaw gap invalidates continuity");
        check(!yaw.ingest(-159,1.1),"lost yaw remains invalid until restart");
        ImuParser ascii("ascii");std::string line="noise\nYAW,12.5\nYAW,nan\n";
        auto angles=ascii.feed(reinterpret_cast<const uint8_t*>(line.data()),line.size());
        check(angles.size()==1 && angles[0]==12.5,"ascii parsing");
        uint8_t packet[11]={0x55,0x53,0,0,0,0,0,0x40,0,0,0};
        for(int k=0;k<10;++k) packet[10]=uint8_t(packet[10]+packet[k]);
        ImuParser binary("wit11");check(binary.feed(packet,5).empty(),"partial packet");
        angles=binary.feed(packet+5,6);check(angles.size()==1&&angles[0]==90,"binary yaw/checksum");
        packet[10]^=1;check(binary.feed(packet,11).empty(),"invalid checksum");
        Vision vision(p);auto image=road();rectangle(image,156,20,164,240,{0,0,0});
        checkGuideContrastAndIdentity(p);
        auto o=vision.analyze(image,Stage::Depart);
        check(o.frameValid&&o.blackPath.confidence>.9,"single black centerline");
        check(std::abs(o.blackPath.lateral)<.01,"centerline error");check(!o.stripe,"single line is not zebra");
        check(o.cones.empty(),"blue background is not cone");
        image=road();rectangle(image,118,125,134,160,{0,90,210});o=vision.analyze(image,Stage::Cones);
        check(o.cones.size()==1,"enclosed cone");auto avoid=vision.avoidCones(o,image);
        check(avoid.valid()&&avoid.x[158]>.55,"cone avoidance to opposite side");
        image=road();rectangle(image,130,100,190,154,{0,90,210});o=vision.analyze(image,Stage::ToRing);
        check(o.ringCandidate,"enclosed roundabout island");
        check(o.cones.empty(),"roundabout island is not also a cone");
        check(o.ringIslands.size()==1,"roundabout island remains an approach obstacle");
        avoid=vision.avoidCones(o,image);
        check(avoid.valid() && std::abs(avoid.x[152]-.5)>.2,"approaching ring retains obstacle clearance");
        rectangle(image,220,165,236,200,{0,90,210});o=vision.analyze(image,Stage::Cones);
        check(o.ringCandidate && o.cones.size()==1,"separate cone remains detected alongside ring island");
        image=road();for(int x=90;x<230;x+=24) rectangle(image,x,180,x+10,215,{0,0,0});
        o=vision.analyze(image,Stage::ToCross);check(o.stripe&&o.stripeY>.85,"zebra nearest edge");
        image=road();rectangle(image,80,35,240,75,{200,20,20});o=vision.analyze(image,Stage::CrossWait);
        check(o.bar&&o.barObservable,"coloured pedestrian board");
        image=road();rectangle(image,85,80,105,225,{0,90,210});rectangle(image,215,80,235,225,{0,90,210});
        o=vision.analyze(image,Stage::ToGarage);check(o.garage&&std::abs(o.garageError)<.01,"garage walls");
        o=vision.analyze(Image(320,240),Stage::Depart);check(!o.frameValid,"dark disconnected camera");
        checkBlackReacquisition(p);
        checkConeRingTransition(p);
        Simulation s(p);s.step();s.step(.36);check(s.mission.stage()==Stage::ToCones,"departure");
        s.reachRing();s.finishRing();s.crossing(true);check(s.mission.completedLaps()==1,"first lap count");
        check(s.mission.stage()==Stage::ToCones,"first lap must not park");
        s.reachRing();s.finishRing();s.crossing(false);check(s.mission.completedLaps()==2,"second lap count");
        check(s.mission.stage()==Stage::GarageAlign,"final parking start");
        s.step(.05,-30);s.repeat(3,.04);check(s.mission.stage()==Stage::GarageAdvance,"align outward axis");
        s.step(.26,0,0);check(s.mission.stage()==Stage::GarageReverse,"stop before reverse");
        s.step(.03,0,-.05);check(s.command.speed<0,"reverse speed sign");
        s.step(.33,0,0);check(s.mission.stage()==Stage::Complete&&s.command.speed==0,"park complete stop");
        Simulation fail(p);fail.o.frameValid=false;
        fail.command=fail.mission.update(fail.o,fail.t,good());check(fail.command.stage==Stage::Fault&&fail.command.speed==0,"camera fault stop");
        Simulation unknown(p);unknown.step();unknown.step(.36);unknown.reachRing();unknown.finishRing();
        unknown.o.stripe=true;unknown.o.stripeY=.82;unknown.o.barObservable=false;unknown.repeat(4);
        check(unknown.command.speed==0&&unknown.mission.stage()==Stage::CrossApproach,"unknown bar must not pass");
        Params clockwise=p;clockwise.ring_side=1;Simulation cw(clockwise);
        cw.step();cw.step(.36);cw.reachRing();cw.finishRing();
        check(cw.mission.stage()==Stage::ToCross,"clockwise ring also exits");
        Simulation stale(p);stale.step();stale.o.blackPath=Path{};
        stale.step(0,0,.1,.1);check(stale.command.speed<=p.cross_speed,"short line gap slows down");
        stale.t.time+=1;auto stopped=stale.mission.update(stale.o,stale.t,good());
        check(stopped.stage==Stage::Fault&&stopped.speed==0,"long line gap stops");
        Simulation backwards(p);backwards.step(.1);backwards.t.distance=0;
        stopped=backwards.mission.update(backwards.o,backwards.t,good());
        check(stopped.stage==Stage::Fault,"odometry cannot move backwards during reverse");
        Simulation noYaw(p);noYaw.step();noYaw.step(.36);noYaw.reachRing();
        noYaw.t.yawValid=noYaw.t.yawMeasured=false;
        stopped=noYaw.mission.update(noYaw.o,noYaw.t,good());
        check(stopped.stage==Stage::Fault&&stopped.speed==0,"ring sensor loss stops");
        std::cout<<"PASS "<<checks<<" checks, including complete two-lap mission\n";return 0;
    }catch(const std::exception& e) {std::cerr<<"FAIL after "<<checks<<" checks: "<<e.what()<<'\n';return 1;}
}
