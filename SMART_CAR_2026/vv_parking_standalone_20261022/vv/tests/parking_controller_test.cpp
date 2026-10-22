#include "Parking/ParkingRuntime.hpp"
#include "Parking/SteeringDiagnostics.hpp"
#include "Parking/SteeringUpdate.hpp"
#include "Contral/Steer/SteerPwmSequence.hpp"
#include <atomic>
#include <cassert>
#include <iostream>
#include <thread>
#include <vector>

using namespace Parking;
namespace {
struct FakeSteeringPwm {
    int period=0, duty=0;
    bool enabled=false;
    char failAt=0;
    std::vector<char> operations;
    bool setPeriod(unsigned int value) {
        operations.push_back('P');
        if(failAt=='P') return false;
        assert(value>0 && value>=static_cast<unsigned int>(duty));
        period=static_cast<int>(value); return true;
    }
    bool disable() {
        operations.push_back('D');
        // Model the real regression: a newly exported zero-period channel
        // cannot be disabled before a valid period has been configured.
        if(failAt=='D' || period==0) return false;
        enabled=false; return true;
    }
    bool enable() {
        operations.push_back('E');
        assert(period>0 && duty>0 && duty<period);
        if(failAt=='E') return false;
        enabled=true; return true;
    }
    bool writeAngle(float angle,float offset) {
        operations.push_back('W');
        assert(period>0 && !enabled);
        if(failAt=='W') return false;
        duty=static_cast<int>(steeringDutyNs(angle,offset)); return true;
    }
};
void verifySteeringRestart() {
    FakeSteeringPwm pwm; SteeringUpdate update;
    auto apply=[&](float angle,float offset=95) {
        return update.apply(angle,offset,[&] {
            return Contral::restartSteeringPwm(pwm,3040000,[&] { return pwm.writeAngle(angle,offset); });
        });
    };
    assert(apply(102)==SteeringUpdateResult::Applied);
    assert(pwm.operations==std::vector<char>({'P','D','W','E'}));
    assert(pwm.enabled && pwm.duty==1700711);
    for(int i=0;i<500;++i) assert(apply(102)==SteeringUpdateResult::Unchanged);
    assert(pwm.operations.size()==4); // HOLD does not disable or restart PWM
    assert(apply(-5)==SteeringUpdateResult::Applied && pwm.duty==1520000);
    assert(apply(102)==SteeringUpdateResult::Applied && pwm.duty==1700711);
    assert(apply(220)==SteeringUpdateResult::Applied && pwm.duty==1900000);
    assert(apply(102)==SteeringUpdateResult::Applied);
    const auto beforeHold=pwm.operations.size();
    for(int phase=0;phase<3;++phase) for(int i=0;i<100;++i)
        assert(apply(102)==SteeringUpdateResult::Unchanged); // S5/DONE/FAULT use same center
    assert(pwm.operations.size()==beforeHold);
    assert(apply(102,94)==SteeringUpdateResult::Applied); // offset is part of the target

    const char failureSteps[]={'P','D','W','E'};
    for(int i=0;i<4;++i) {
        FakeSteeringPwm bad; bad.failAt=failureSteps[i]; SteeringUpdate once;
        auto write=[&] { return Contral::restartSteeringPwm(bad,3040000,
            [&] { return bad.writeAngle(102,95); }); };
        assert(once.apply(102,95,write)==SteeringUpdateResult::Failed);
        assert(bad.operations.size()==static_cast<std::size_t>(i+1));
        assert(!bad.enabled);
        for(int frame=0;frame<500;++frame)
            assert(once.apply(220,95,write)==SteeringUpdateResult::Failed);
        assert(bad.operations.size()==static_cast<std::size_t>(i+1)); // no automatic retry
        Controller stopped; stopped.arm(Config{},0); stopped.stop("steering write failed");
        auto out=stopped.tick(0);
        assert(out.phase==Phase::Fault && out.leftPwm==0 && out.rightPwm==0);
    }
    SteeringUpdate mismatch; int calls=0;
    assert(mismatch.apply(102,95,[&] { ++calls; return false; })==SteeringUpdateResult::Failed);
    assert(mismatch.apply(102,95,[&] { ++calls; return true; })==SteeringUpdateResult::Failed);
    assert(calls==1); // kernel verification failure also latches the write failure

    // Exercise the real controller and update gate together. S2 -> S4 -> center
    // needs exactly three restarts; motor start, S5 and DONE must not add any.
    Config route; route.approachMs=0;
    route.stages[0]={true,-107,12600,3000,2500};
    route.stages[1]={true,118,12000,2000,1000};
    Controller flow; assert(flow.arm(route,0));
    SteeringUpdate routeUpdate; FakeSteeringPwm routePwm;
    for(int time=0;time<=7000;time+=20) {
        const auto command=flow.tick(time);
        assert(command.ownsActuators);
        const auto result=routeUpdate.apply(command.angle,route.servoOffset,[&] {
            return Contral::restartSteeringPwm(routePwm,3040000,
                [&] { return routePwm.writeAngle(command.angle,route.servoOffset); });
        });
        assert(result!=SteeringUpdateResult::Failed);
        if(time>=6000) assert(command.phase==Phase::Done && command.rightPwm==0);
    }
    assert(routePwm.operations.size()==12 && routePwm.enabled && routePwm.duty==1700711);
}
}
int main(int argc, char **argv) {
    verifySteeringRestart();
    // A repeat on lap one and any zebra on lap three cannot arm parking.
    Mailbox box; Request request, taken;
    assert(!box.submit(0,request)); assert(!box.submit(0,request));
    assert(!box.submit(2,request)); assert(box.submit(1,request));
    assert(!box.submit(1,request)); assert(box.take(taken)); assert(!box.take(taken));
    assert(!box.submit(1,request)); box.reset(); assert(box.submit(1,request));
    assert(box.submitted()); assert(box.take(taken));
    assert(box.submitted()); // consuming the request must not re-enable zebra events

    Mailbox events; int detectorCalls=0;
    auto detect=[&] { ++detectorCalls; return true; };
    auto event=detectZebraEvent(events,false,detect);
    assert(!event.triggered && !event.voiceAllowed && detectorCalls==0);
    event=detectZebraEvent(events,true,[] { return false; });
    assert(!event.triggered && !event.voiceAllowed);
    event=detectZebraEvent(events,true,detect);
    assert(event.triggered && event.voiceAllowed && detectorCalls==1);
    assert(!events.submit(0,request)); // ordinary first-lap events remain enabled
    event=detectZebraEvent(events,true,detect);
    assert(event.triggered && event.voiceAllowed && detectorCalls==2);
    assert(events.submit(1,request));
    assert(event.voiceAllowed); // current event's permission survives same-frame submission
    for (int frame=0;frame<500;++frame) {
        auto noise=detectZebraEvent(events,true,detect);
        assert(!noise.triggered && !noise.voiceAllowed && detectorCalls==2);
        if(frame==5) assert(events.take(taken));
    }
    events.reset();
    assert(detectZebraEvent(events,true,detect).voiceAllowed && detectorCalls==3);
    assert(events.submit(1,request));
    auto assertNoZebra=[&] {
        for(int frame=0;frame<500;++frame) {
            const auto noise=detectZebraEvent(events,true,detect);
            assert(!noise.triggered && !noise.voiceAllowed && detectorCalls==3);
        }
    };

    // Use the user's measured left/center/right commands with unchanged timing.
    // then center only after 1000ms of driving, not from stage entry.
    Config tuned; tuned.approachMs=3500;
    tuned.stages[0]={true,-107,12600,3000,2500};
    tuned.stages[1]={true,118,12000,2000,1000};
    Controller actual; assert(actual.arm(tuned,3200));
    assert(actual.tick(0).phase==Phase::StopHold); assertNoZebra();
    assert(actual.tick(3200).phase==Phase::Approach);
    assertNoZebra();
    actual.forwardApplied(3200);
    assert(actual.tick(6699).phase==Phase::Approach);
    auto tunedOut=actual.tick(6700);
    assert(tunedOut.phase==Phase::S2 && tunedOut.angle==-5 && tunedOut.rightPwm==0);
    assertNoZebra();
    assert(actual.tick(7200).rightPwm==-12600);
    assert(actual.tick(9699).angle==-5);
    tunedOut=actual.tick(9700);
    assert(tunedOut.phase==Phase::S4 && tunedOut.angle==220 && tunedOut.rightPwm==0);
    assertNoZebra();
    assert(actual.tick(10199).rightPwm==0);
    tunedOut=actual.tick(10200);
    assert(tunedOut.angle==220 && tunedOut.rightPwm==-12000);
    assert(actual.tick(11199).angle==220);
    tunedOut=actual.tick(11200);
    assert(tunedOut.angle==102 && tunedOut.rightPwm==-12000);
    assert(actual.tick(11700).phase==Phase::S5);
    assertNoZebra();
    assert(actual.tick(12200).rightPwm==-9000);
    assert(actual.tick(12700).phase==Phase::Done);
    assertNoZebra();
    assert(box.submitted());
    actual.stop("test fault"); assert(actual.tick(13000).phase==Phase::Fault);
    assertNoZebra();
    assert(box.submitted());
    assert(static_cast<int>(steeringDutyNs(-95,95))==1368000);
    assert(static_cast<int>(steeringDutyNs(-5,95))==1520000);
    assert(static_cast<int>(steeringDutyNs(55,95))==1621333);
    assert(static_cast<int>(steeringDutyNs(102,95))==1700711);
    assert(static_cast<int>(steeringDutyNs(220,95))==1900000);
    SteeringPwmSnapshot snapshot;
    assert(std::string(snapshot.status())=="INVALID_COMMAND");
    snapshot.expectedDutyNs=1621333;
    assert(std::string(snapshot.status())=="READ_FAILED");
    snapshot.sysfsDutyNs=1621333; snapshot.sysfsPeriodNs=3040000; snapshot.sysfsEnabled=1;
    assert(std::string(snapshot.status())=="MATCH");
    snapshot.sysfsDutyNs=1368000;
    assert(std::string(snapshot.status())=="MISMATCH");
    snapshot.sysfsPeriodNs=20000000;
    assert(std::string(snapshot.status())=="PERIOD_MISMATCH");
    snapshot.sysfsEnabled=0;
    assert(std::string(snapshot.status())=="PWM_DISABLED");
    if (argc==2) {
        const std::string directory=argv[1];
        std::ofstream(directory+"/duty_cycle") << 1621333;
        std::ofstream(directory+"/period") << 3040000;
        std::ofstream(directory+"/enable") << 1;
        assert(std::string(inspectSteeringPwm(55,95,directory).status())=="MATCH");
        std::ofstream(directory+"/duty_cycle") << 1368000;
        assert(std::string(inspectSteeringPwm(55,95,directory).status())=="MISMATCH");
        std::ofstream(directory+"/duty_cycle") << "invalid";
        assert(std::string(inspectSteeringPwm(55,95,directory).status())=="READ_FAILED");
        assert(std::string(inspectSteeringPwm(55,95,directory+"/missing").status())=="READ_FAILED");
    }

    // Retain historical timing regressions with their original explicit inputs.
    Config c; c.center=-5; c.stages[0].steerOffset=-142; c.stages[1].steerOffset=5;
    Controller flow;
    assert(flow.arm(c,3200)); assert(!flow.arm(c,0));
    assert(!flow.tick(3199).ownsActuators);
    assert(flow.tick(3200,false).phase == Phase::StopHold);
    assert(flow.tick(3250).phase == Phase::Approach);
    // Waiting for a real forward command must not consume the 4000ms window.
    assert(!flow.tick(9000).ownsActuators);
    flow.forwardApplied(9000); flow.forwardApplied(9500);
    assert(!flow.tick(12999).ownsActuators);
    auto out=flow.tick(13000);
    assert(out.phase==Phase::S2 && out.ownsActuators && out.rightPwm==0 && out.angle==-147);
    assert(flow.tick(13499).rightPwm==0);
    out=flow.tick(13500); assert(out.leftPwm==0 && out.rightPwm==-12000 && out.angle==-147);
    assert(flow.tick(14899).angle==-147);
    assert(flow.tick(14900).angle==-5 && flow.tick(14900).rightPwm==-12000);
    out=flow.tick(15250); assert(out.phase==Phase::S4 && out.rightPwm==0 && out.angle==0);
    assert(flow.tick(15750).rightPwm==-12000);
    assert(flow.tick(16750).angle==-5);
    out=flow.tick(17250); assert(out.phase==Phase::S5 && out.rightPwm==0 && out.angle==-5);
    assert(flow.tick(17750).rightPwm==-9000);
    out=flow.tick(18250); assert(out.phase==Phase::Done && out.rightPwm==0 && out.leftPwm==0);
    assert(flow.tick(100000).phase==Phase::Done && flow.tick(100000).ownsActuators);

    // Exhaustively exercise every stage enable/skip combination. Each stage
    // has distinctive timing/PWM/steering so stale initialization is observable.
    for(int mask=0;mask<8;++mask) {
        Config p; p.approachMs=0; p.settleMs=10;
        p.leftEnabled=true; p.rightEnabled=false;
        for(int i=0;i<3;++i) p.stages[i]={(mask&(1<<i))!=0,10+i,1000+i,100+20*i,0};
        Controller controller; assert(controller.arm(p,100));
        p.stages[0].pwm=49999; // changing the caller's config cannot alter the snapshot
        std::int64_t time=100; auto cmd=controller.tick(time);
        const Phase phases[]={Phase::S2,Phase::S4,Phase::S5};
        for(int i=0;i<3;++i) if(mask&(1<<i)) {
            assert(cmd.phase==phases[i] && cmd.leftPwm==0 && cmd.rightPwm==0);
            cmd=controller.tick(time+10);
            assert(cmd.leftPwm==-(1000+i) && cmd.rightPwm==0 && cmd.angle==p.center+10+i);
            assert(controller.tick(time+99+20*i).angle==p.center+10+i); // hold=0
            time+=100+20*i; cmd=controller.tick(time);
        }
        assert(cmd.phase==Phase::Done && cmd.leftPwm==0 && cmd.rightPwm==0);
    }
    Config zeroWait; zeroWait.approachMs=0; zeroWait.settleMs=0;
    Controller immediate; immediate.arm(zeroWait,0);
    assert(immediate.tick(0).rightPwm==0);
    assert(immediate.tick(1).rightPwm==-12000);
    immediate.stop("manual stop"); out=immediate.tick(2);
    assert(out.phase==Phase::Fault && out.rightPwm==0 && out.angle==zeroWait.center);
    assert(immediate.tick(10000).phase==Phase::Fault);
    Config disabled; disabled.enabled=false; disabled.approachMs=0;
    Controller off; off.arm(disabled,0); assert(off.tick(0).phase==Phase::Done);
    for(int invalid=0;invalid<5;++invalid) {
        Config bad;
        if(invalid==0) bad.stages[0].totalMs=bad.settleMs;
        if(invalid==1) bad.stages[1].holdMs=20000;
        if(invalid==2) bad.stages[2].pwm=0;
        if(invalid==3) bad.leftEnabled=bad.rightEnabled=false;
        if(invalid==4) bad.center=10000;
        Controller reject; reject.arm(bad,0); out=reject.tick(0);
        assert(out.phase==Phase::Fault && out.leftPwm==0 && out.rightPwm==0);
        assert(!reject.fault().empty());
    }
    // Run a normal writer concurrently with handover, then prove every normal
    // write after parking ownership is rejected (including after completion).
    SteeringAuthority authority; std::atomic<bool> started{false},finish{false};
    int lastCommand=0;
    std::thread image([&]{ started=true; while(!finish.load())
        authority.normal([&]{lastCommand=90;}); });
    while(!started.load()) std::this_thread::yield();
    authority.parking([&]{lastCommand=-147;});
    for(int i=0;i<10000;++i) assert(!authority.normal([&]{lastCommand=90;}));
    finish=true; image.join(); assert(lastCommand==-147);
    authority.parking([&]{lastCommand=-5;});
    assert(!authority.normal([&]{lastCommand=90;})); assert(lastCommand==-5);
    std::cout << "PASS parking: second-lap only, hold/approach clock, all 8 stage masks, "
        "exact stop boundaries, snapshot, validation, manual stop, concurrent steering ownership, "
        "measured left/center/right timing, PWM math, period-zero initialization, "
        "target-change-only restarts, restart failures and diagnostic states, "
        "zebra event/voice gate across submission/take/reset\n";
}
