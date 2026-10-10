#include "../tools/parking_rehearsal.hpp"
#include "../tools/rehearsal_session.hpp"
#include <chrono>
#include <iostream>
#include <limits>
using namespace car2026::capture;
int checks=0;
void check(bool ok,const char* name) {++checks;if(!ok) throw std::runtime_error(name);}
template<class F> void rejects(F action,const char* name) {bool failed=false;try {action();}catch(const std::exception&) {failed=true;}check(failed,name);}
std::string config(bool single=true,int stage=2) {
    std::string result="[session]\nmode="+std::string(single?"single":"full")+"\nstage="+std::to_string(stage)+"\n";
    for(int i=1;i<=6;++i) result+="[stage_"+std::to_string(i)+"]\nsteer_command="+std::to_string(i==2?-10:i==4?10:0)+"\nsettle_time_s=0.5\nhold_time_s=5\n";
    return result;
}
void start(ParkingRehearsal& model,double time) {
    auto request=model.update(time,RehearsalKey::Enter);check(request.has_value(),"apply requires Enter");
    check(model.recordingEnabled(),"one Enter starts recording before servo settling");
    model.acknowledge(true,time);check(!model.update(time+.49,RehearsalKey::Enter),"settling discards Enter");
    model.update(time+.5,RehearsalKey::None);
    check(model.state()==ParkingRehearsal::State::Running,"one authorization automatically runs after settling without another Enter");
}
int main() {
 try {
    const auto initial=ParkingTuning::parse(config());check(initial.stage==2 && initial.single,"single stage2 parsed");
    check(initial.stages[1].speed.leftRps==9 && initial.stages[1].speed.kp==64 && initial.stages[1].speed.pwmLimit==12000,"older file gets cc speed defaults without treating raw command as rps");
    auto motorText=config();motorText.insert(motorText.find("[stage_3]"),"motor_left_command=3000\nmotor_right_command=3000\nmotor_run_time_s=7\nmotor_left_target_rps=5\nmotor_right_target_rps=6\nmotor_pid_pwm_limit=50000\n");
    const auto motorTuning=ParkingTuning::parse(motorText);motorTuning.validateReferenceSpeed(50000);
    check(motorTuning.stages[1].speed.leftRps==5 && motorTuning.stages[1].speed.rightRps==6 && motorTuning.stages[1].speed.pwmLimit==50000,"independent reference speed and full PWM range parsed");
    rejects([&]{motorTuning.validateReferenceSpeed(49999);},"device period also constrains PID mode");
    ParkingRehearsal newSpeedTrial(motorTuning);start(newSpeedTrial,0);
    auto changedSpeed=motorTuning;changedSpeed.stages[1].speed.leftRps=7;
    check(newSpeedTrial.applySaved(changedSpeed,1).has_value(),"saving active speed starts a new authorized trial");
    auto invalidText=motorText;invalidText.replace(invalidText.find("motor_pid_pwm_limit=50000"),25,"motor_pid_pwm_limit=50001");
    SavedTuningWatcher modeWatcher(motorTuning,15,[](const ParkingTuning& value){value.validateReferenceSpeed(50000);});
    modeWatcher.observe(invalidText,0);
    rejects([&]{modeWatcher.observe(invalidText,.4);},"invalid mode save is rejected before accepted snapshot changes");
    modeWatcher.observe(motorText,.5);
    check(!modeWatcher.observe(motorText,.9),"restoring original valid mode file does not re-authorize motion");
    auto combinedText=config();combinedText.insert(combinedText.find("[stage_1]"),"motor_target_rps=4\n");
    const auto combined=ParkingTuning::parse(combinedText);
    check(combined.stages[0].speed.leftRps==4 && combined.stages[0].speed.rightRps==4 && combined.stages[4].speed.leftRps==4,"one session speed parameter controls both wheels in all stages");
    auto onlySpeedText=combinedText;
    onlySpeedText.insert(onlySpeedText.find("[stage_2]"),"motor_run_time_s=7\n");
    const auto onlySpeed=ParkingTuning::parse(onlySpeedText,true);
    check(onlySpeed.ccSpeedControl && onlySpeed.stages[0].powered(),"CC duration alone enables both wheels without raw PWM keys");
    for(size_t i=0;i<5;++i) check(onlySpeed.stages[i].motorLeft==1 && onlySpeed.stages[i].motorRight==1 &&
        onlySpeed.stages[i].speed.leftRps==4 && onlySpeed.stages[i].speed.rightRps==4,"all driving stages share one speed and automatic wheel enable");
    check(!onlySpeed.stages[1].powered() && !onlySpeed.stages[5].powered(),"zero duration and stop-confirmation cannot start motors");
    rejects([&]{ParkingTuning::parse(onlySpeedText);},"duration-only speed file cannot accidentally activate raw PWM mode");
    auto oldSpeedText=onlySpeedText;
    oldSpeedText.insert(oldSpeedText.find("[stage_2]"),"motor_left_command=3000\nmotor_right_command=0\n");
    const auto oldSpeed=ParkingTuning::parse(oldSpeedText,true);
    check(oldSpeed.stages[0].motorLeft==1 && oldSpeed.stages[0].motorRight==1,"legacy raw values cannot set CC speed or disable one wheel");
    const auto upgraded=normalizeCcSpeedSource(oldSpeedText);
    check(upgraded.find("motor_left_command=")==std::string::npos && upgraded.find("motor_right_command=")==std::string::npos,"upgrade removes all legacy wheel keys");
    check(upgraded.find("motor_target_rps=4")!=std::string::npos && upgraded.find("motor_run_time_s=7")!=std::string::npos,"upgrade preserves common speed and successful duration");
    const auto upgradedTuning=ParkingTuning::parse(upgraded,true);
    for(size_t i=0;i<6;++i) check(upgradedTuning.stages[i].command==oldSpeed.stages[i].command &&
        upgradedTuning.stages[i].holdSeconds==oldSpeed.stages[i].holdSeconds && upgradedTuning.stages[i].motorSeconds==oldSpeed.stages[i].motorSeconds,
        "upgrade keeps every stage steering command and both duration types");
    check(normalizeCcSpeedSource(upgraded)==upgraded,"upgrade is idempotent and does not rewrite an already upgraded file");
    auto oldPd=onlySpeedText;
    oldPd.replace(oldPd.find("hold_time_s=5"),13,"hold_time_s=0");
    oldPd.insert(oldPd.find("[stage_2]"),"line_kp=0.35\nline_kd=0.02\nline_target_x=79.5\nline_crop_top=0.30\nline_max_command=8\nline_follow_enable=1\n");
    const auto ccLaneUpgrade=normalizeCcSpeedSource(oldPd);
    check(ccLaneUpgrade.find("line_kp=")==std::string::npos && ccLaneUpgrade.find("line_kd=")==std::string::npos &&
        ccLaneUpgrade.find("line_target_x=")==std::string::npos && ccLaneUpgrade.find("line_crop_top=")==std::string::npos,
        "CC upgrade removes unused PD and old optical target parameters");
    const auto ccLaneTuning=ParkingTuning::parse(ccLaneUpgrade,true);
    check(ccLaneTuning.stages[0].line.enabled && ccLaneTuning.stages[0].line.maxCommand==8 &&
        ccLaneTuning.stages[0].motorSeconds==7 && ccLaneTuning.referenceSpeed.leftRps==4,
        "CC upgrade preserves line enable, mechanical limit, speed and successful stage duration");
    check(normalizeCcSpeedSource(ccLaneUpgrade)==ccLaneUpgrade,"CC lane upgrade is idempotent");
    auto withoutTarget=oldSpeedText;withoutTarget.erase(withoutTarget.find("motor_target_rps=4\n"),19);
    rejects([&]{ParkingTuning::parse(withoutTarget,true);},"missing active speed key cannot silently fall back to a faster default");
    check(normalizeCcSpeedSource(withoutTarget).find("motor_target_rps=9")!=std::string::npos,"old file gets one visible default speed inside session");
    check(ParkingTuning::parse(normalizeCcSpeedSource("\xef\xbb\xbf"+withoutTarget),true).referenceSpeed.leftRps==9,"BOM upgrade remains valid");
    SavedTuningWatcher oneSpeedWatcher(onlySpeed,15);
    auto oneSpeedChanged=onlySpeedText;oneSpeedChanged.replace(oneSpeedChanged.find("motor_target_rps=4"),18,"motor_target_rps=6");
    oneSpeedWatcher.observe(oneSpeedChanged,0);const auto oneSpeedReload=oneSpeedWatcher.observe(oneSpeedChanged,.4);
    check(oneSpeedReload && oneSpeedReload->ccSpeedControl && oneSpeedReload->stages[0].powered() &&
        oneSpeedReload->stages[4].speed.leftRps==6,"file reload retains CC parsing mode and updates all stages together");
    for(const auto& extra:{"motor_left_target_rps=2\n","motor_right_target_rps=2\n"}) {
        auto mixed=onlySpeedText;mixed.insert(mixed.find("[stage_1]"),extra);
        rejects([&]{ParkingTuning::parse(mixed,true);},"CC mode rejects competing separate speed knobs");
    }
    auto stageOverride=onlySpeedText;stageOverride.insert(stageOverride.find("[stage_2]"),"motor_target_rps=2\n");
    rejects([&]{ParkingTuning::parse(stageOverride,true);},"CC mode rejects stage override of global speed");
    auto stoppedTime=onlySpeedText;stoppedTime+="motor_run_time_s=1\n";
    rejects([&]{ParkingTuning::parse(stoppedTime,true);},"stop-confirmation positive duration is forbidden in CC mode");
    rejects([&]{normalizeCcSpeedSource(oldSpeedText+"unknown=1\n");},"invalid old configuration is rejected before upgrade");
    auto oneSpeedLine=onlySpeedText;oneSpeedLine.replace(oneSpeedLine.find("hold_time_s=5"),13,"hold_time_s=0");
    oneSpeedLine.insert(oneSpeedLine.find("[stage_2]"),"line_follow_enable=1\n");
    check(ParkingTuning::parse(oneSpeedLine,true).stages[0].line.enabled,"line following works without wheel PWM fields");
    auto combinedChange=combinedText;combinedChange.replace(combinedChange.find("motor_target_rps=4"),18,"motor_target_rps=5");
    ParkingRehearsal combinedTrial(combined);
    check(combinedTrial.applySaved(ParkingTuning::parse(combinedChange),0).has_value(),"one global speed save re-authorizes current stage");
    check(initial.speedLeftCmPerCount==0 && initial.speedRightCmPerCount==0,"legacy config only produces counts/s");
    auto speedText=config();speedText.insert(speedText.find("[stage_1]"),"speed_left_cm_per_count=0.01\nspeed_right_cm_per_count=0.02\n");
    const auto speedTuning=ParkingTuning::parse(speedText);
    check(speedTuning.speedLeftCmPerCount==.01 && speedTuning.speedRightCmPerCount==.02,"explicit independent speed calibration parsed");
    for(double scale:{-1.,11.,std::numeric_limits<double>::quiet_NaN()}) {
        auto badSpeed=initial;badSpeed.speedLeftCmPerCount=scale;
        rejects([&] {badSpeed.validate();},"invalid speed calibration rejected");
    }
    ParkingRehearsal noActuation(initial);
    check(!noActuation.applySaved(speedTuning,0) && !noActuation.recordingEnabled(),"calibration-only save never authorizes steering or starts data");
    SavedTuningWatcher speedWatcher(initial,15);speedWatcher.observe(speedText,0);
    check(speedWatcher.observe(speedText,.4).has_value(),"watcher notices speed scale without losing snapshot");
    ParkingRehearsal pausedSettle(initial);
    pausedSettle.update(0,RehearsalKey::Enter);pausedSettle.acknowledge(true,0);
    pausedSettle.update(.1,RehearsalKey::Pause);pausedSettle.update(.5,RehearsalKey::None);
    check(pausedSettle.state()==ParkingRehearsal::State::Running && !pausedSettle.recordingEnabled(),"P during settle only pauses data, authorized timer can start");
    pausedSettle.update(.6,RehearsalKey::Continue);
    check(pausedSettle.state()==ParkingRehearsal::State::Running && pausedSettle.recordingEnabled(),"C resumes same trial without second permission");
    check(ParkingTuning::parse("\xef\xbb\xbf"+config()).stage==2,"UTF8 BOM accepted");
    check(initial.stages[1].correction.holdSeconds==0,"older tuning files do not silently enable a new nonzero command");
    auto narrowLegacy=initial;for(auto& stage:narrowLegacy.stages) stage.command=0;
    narrowLegacy.validate(4);
    check(narrowLegacy.stages[1].correction.command==0,"disabled legacy correction does not introduce a command above a narrow vehicle limit");
    auto correctionText=config();correctionText.insert(correctionText.find("[stage_3]"),"correction_steer_command=5\ncorrection_hold_time_s=1\n");
    const auto withCorrection=ParkingTuning::parse(correctionText);
    check(withCorrection.stages[1].correction.command==5 && withCorrection.stages[1].correction.holdSeconds==1,"right correction parameters parsed");
    for(const auto& entry:{std::string("correction_steer_command=5\n"),std::string("correction_hold_time_s=1\n")}) {
        auto partial=config();partial.insert(partial.find("[stage_3]"),entry);
        rejects([&] {ParkingTuning::parse(partial);},"partially specified correction rejected");
    }
    auto unsafeCorrection=withCorrection;unsafeCorrection.stages[1].correction.command=-5;
    rejects([&] {unsafeCorrection.validate();},"left correction forbidden for this right correction action");
    unsafeCorrection=withCorrection;unsafeCorrection.stages[1].command=5;
    rejects([&] {unsafeCorrection.validate();},"first bend must remain opposite to correction");
    unsafeCorrection=withCorrection;unsafeCorrection.stages[1].holdSeconds=0;
    rejects([&] {unsafeCorrection.validate();},"enabled correction requires timed first bend");
    auto missingCore=correctionText;missingCore.erase(missingCore.find("mode=single\n"),12);
    rejects([&] {ParkingTuning::parse(missingCore);},"optional keys cannot replace a required core key");
    auto correctionOnlyText=correctionText;correctionOnlyText.insert(correctionOnlyText.find("[stage_1]"),"stage_2_step=right_correction\n");
    const auto correctionOnly=ParkingTuning::parse(correctionOnlyText);
    check(correctionOnly.stages[1].correctionOnly,"correction-only selection parsed");
    auto invalidSelection=correctionOnly;invalidSelection.single=false;
    rejects([&] {invalidSelection.validate();},"correction-only cannot skip main bend in full mode");
    invalidSelection=correctionOnly;invalidSelection.stage=4;
    rejects([&] {invalidSelection.validate();},"correction-only is limited to stage2");
    invalidSelection=correctionOnly;invalidSelection.stages[1].correction.holdSeconds=0;
    rejects([&] {invalidSelection.validate();},"correction-only requires positive hold time");
    auto disabled=withCorrection;disabled.stages[1].correction.holdSeconds=0;disabled.validate();
    check(disabled.stages[1].correction.holdSeconds==0,"zero correction duration disables automatic extra command");
    for(double value:{-1.,121.,std::numeric_limits<double>::quiet_NaN()}) {
        auto invalid=withCorrection;invalid.stages[1].correction.holdSeconds=value;
        rejects([&] {invalid.validate();},"invalid correction duration rejected");
    }
    for(double value:{0.,16.,std::numeric_limits<double>::quiet_NaN()}) {
        auto invalid=withCorrection;invalid.stages[1].correction.command=value;
        rejects([&] {invalid.validate();},"invalid enabled correction command rejected");
    }
    auto unknownStep=correctionText;unknownStep.insert(unknownStep.find("[stage_1]"),"stage_2_step=automatic\n");
    rejects([&] {ParkingTuning::parse(unknownStep);},"unknown correction selector rejected");
    rejects([] {ParkingTuning::parse(config()+"steer_command=1\n");},"duplicate key rejected");
    rejects([] {ParkingTuning::parse(config()+"[stage_2]\n");},"duplicate section rejected");
    rejects([] {ParkingTuning::parse(config()+"unknown=1\n");},"unknown key rejected");
    rejects([] {ParkingTuning::parse("[wrong]\n");},"unknown section rejected");
    rejects([] {ParkingTuning::parse("[session]\nmode=single\nstage=2\n");},"missing six-stage params rejected");
    for(const auto& value:{"nan","inf","16","-16"}) {
        auto text=config();text.replace(text.find("steer_command=-10"),17,"steer_command="+std::string(value));
        rejects([&] {ParkingTuning::parse(text);},"invalid command rejected");
    }
    for(const auto& value:{"0.49","5.01","nan"}) {
        auto text=config();text.replace(text.find("settle_time_s=0.5"),17,"settle_time_s="+std::string(value));
        rejects([&] {ParkingTuning::parse(text);},"invalid settle rejected");
    }
    for(const auto& value:{"-1","120.01","nan"}) {
        auto text=config();text.replace(text.find("hold_time_s=5"),13,"hold_time_s="+std::string(value));
        rejects([&] {ParkingTuning::parse(text);},"invalid hold duration rejected");
    }
    for(int stage:{0,7}) rejects([&] {ParkingTuning::parse(config(true,stage));},"stage range rejected");
    auto text=config();text.replace(text.find("single"),6,"automatic");rejects([&] {ParkingTuning::parse(text);},"unknown mode rejected");
    rejects([&] {initial.validate(5);},"vehicle command ceiling checked");
    check(parseRehearsalInput({"",""})==RehearsalKey::Invalid,"queued Enter rejected");
    check(parseRehearsalInput({"10"})==RehearsalKey::Invalid,"numeric terminal tuning rejected");
    check(parseRehearsalInput({"","Q",""})==RehearsalKey::Quit,"quit overrides queued permissions");
    check(parseRehearsalInput({"","P"})==RehearsalKey::Pause,"pause overrides permission");
    for(const auto& pair:std::vector<std::pair<std::string,RehearsalKey>>{{"",RehearsalKey::Enter},{"P",RehearsalKey::Pause},{"C",RehearsalKey::Continue},{"R",RehearsalKey::Reload},{"Q",RehearsalKey::Quit}})
        check(parseRehearsalInput({pair.first})==pair.second,"named key");
    ParkingRehearsal single(initial);check(single.stage()==1,"direct stage2 without stage1");
    check(!single.recordingEnabled(),"startup waits without recording frames or samples");
    check(!single.update(10000,RehearsalKey::None) && !single.finished(),"wait has no duration finish/write");
    start(single,10001);check(single.trial()==1,"first trial number");
    single.update(20000,RehearsalKey::None);check(!single.finished(),"long running trial never times out");
    single.update(20001,RehearsalKey::Enter);check(single.stage()==1 && single.nextStage()==1,"single never advances");
    start(single,20002);check(single.trial()==2,"repeat increments stage trial");
    single.update(20003,RehearsalKey::Pause);check(single.paused(),"pause selected");
    auto saved=ParkingTuning::parse(config(true,4));saved.stages[3].command=12;
    auto request=single.applySaved(saved,20004);
    check(request && *request==12 && single.stage()==3 && single.revision()==2,"file save directly selects/writes stage");
    check(single.paused(),"file save does not resume recording");
    single.acknowledge(true,20004);
    check(single.state()==ParkingRehearsal::State::Running && single.automaticSavedTrial(),"saved execution starts without second Enter");
    single.update(20005,RehearsalKey::Continue);
    check(!single.paused() && single.state()==ParkingRehearsal::State::Running,"C only resumes saved trial records");
    auto other=saved;other.stages[0].command=3;
    check(!single.applySaved(other,20006) && single.stage()==3,"other-stage save does not execute current stage");
    other.stages[3].holdSeconds=8;
    request=single.applySaved(other,20007);
    check(request && *request==12 && single.trial()==2,"duration edit repeats saved angle with new timer/trial");
    single.acknowledge(true,20007);
    single.holdCompleted(1,1);check(!single.finished(),"stale stage completion ignored");
    single.holdCompleted(3,1);check(!single.finished(),"previous trial completion ignored");
    other.stages[0].command=4;
    single.applySaved(other,20007.1);
    single.update(20007.2,RehearsalKey::Pause);
    single.holdCompleted(3,2);
    check(single.outcome()=="TIMED_TRIAL_COMPLETED","paused saved trial ends even after unrelated config revision");
    check(!single.applySaved(initial,20007.3),"finished timed trial cannot restart by save");
    SavedTuningWatcher watcher(initial,15);
    check(!watcher.observe(initial.source,0),"initial file does not auto execute");
    auto changed=config();changed.replace(changed.find("steer_command=-10"),17,"steer_command=-12");
    check(!watcher.observe(changed,.1) && !watcher.observe(changed,.3),"save stability delay");
    auto candidate=watcher.observe(changed,.5);
    check(candidate && candidate->stages[1].command==-12,"stable save yields validated tuning");
    check(!watcher.observe(changed,1),"same save never repeats");
    auto comments=changed+"# comment only\n";watcher.observe(comments,1.1);
    check(!watcher.observe(comments,1.5),"comment edits do not restart timer");
    watcher.observe("",2);rejects([&] {watcher.observe("",2.4);},"truncated file rejected");
    check(!watcher.observe("",2.5),"invalid save reported only once");
    watcher.observe(changed,3);check(!watcher.observe(changed,3.4),"restore accepted contents does not retrigger");
    auto bad=changed;bad.replace(bad.find("steer_command=-12"),17,"steer_command=-16");
    watcher.observe(bad,4);rejects([&] {watcher.observe(bad,4.4);},"invalid saved angle rejected");
    auto good=changed;good.replace(good.find("steer_command=-12"),17,"steer_command=-13");
    watcher.observe(good,5);check(watcher.observe(good,5.4).has_value(),"valid save recovers after error");
    SavedTuningWatcher capped(initial,10);capped.observe(changed,0);
    rejects([&] {capped.observe(changed,.4);},"live save respects vehicle ceiling");
    ParkingRehearsal validActive(initial);validActive.applySaved(other,0);validActive.acknowledge(true,0);
    const auto revision=validActive.revision();auto invalid=other;invalid.stages[3].command=99;
    rejects([&] {validActive.applySaved(invalid,1);},"invalid saved model config rejected");
    check(validActive.revision()==revision && validActive.command()==12,"invalid config leaves active model intact");
    rejects([&] {watcher.observe(good,1);},"watcher rejects time reversal");
    ParkingRehearsal full(ParkingTuning::parse(config(false,1)));
    for(int i=0;i<6;++i) {
        start(full,10.*i);check(full.stage()==i && full.trial()==1,"stage independent trial count");
        full.update(10.*i+1,RehearsalKey::Enter);
        check(full.stage()==i && full.state()==ParkingRehearsal::State::AwaitApply,"stop cannot change steering/stage");
        check(full.nextStage()==std::min(i+1,5),"full next stage preview");
    }
    auto fullEdit=ParkingTuning::parse(config(false,1));fullEdit.stages[5].holdSeconds=8;
    check(full.applySaved(fullEdit,60).has_value() && full.stage()==5,"unchanged file selector preserves progressed full stage");
    full.acknowledge(true,60);
    full.holdCompleted(5,full.trial());
    check(full.outcome()=="TIMED_TRIAL_COMPLETED","saved trial ends in full mode too");
    ParkingRehearsal manualFull(ParkingTuning::parse(config(false,1)));
    start(manualFull,0);manualFull.holdCompleted(0,manualFull.trial());
    check(!manualFull.finished(),"manual full mode remains available after centered segment");
    ParkingRehearsal timedSingle(initial);start(timedSingle,0);
    timedSingle.holdCompleted(1,timedSingle.trial());
    check(timedSingle.finished(),"manual single trial ends after timer");
    ParkingRehearsal paired(withCorrection);start(paired,0);
    check(paired.segment()==0 && paired.command()==-10 && paired.holdSeconds()==5,"paired trial starts with main left command");
    paired.update(.7,RehearsalKey::Enter);
    check(paired.state()==ParkingRehearsal::State::Running,"Enter cannot skip an active timed pair");
    paired.holdCompleted(1,1,0);
    check(!paired.finished() && paired.pendingCorrection() && paired.segment()==1,"left expiry prepares correction instead of ending trial");
    auto correctionRequest=paired.update(6,RehearsalKey::None);
    check(correctionRequest && *correctionRequest==5 && paired.automaticCorrectionWrite(),"right correction needs no additional Enter");
    paired.acknowledge(true,6);
    check(paired.state()==ParkingRehearsal::State::Running && paired.trial()==1 && paired.holdSeconds()==1,"right phase runs with its own duration in same trial");
    paired.holdCompleted(1,1,0);check(!paired.finished(),"duplicate left event cannot finish active right correction");
    paired.holdCompleted(1,1,1);check(paired.finished(),"single paired trial ends only after final right expiry");
    ParkingRehearsal rightOnly(correctionOnly);start(rightOnly,0);
    check(rightOnly.segment()==1 && rightOnly.command()==5 && rightOnly.holdSeconds()==1,"standalone correction skips left bend");
    rightOnly.holdCompleted(1,1,1);check(rightOnly.finished(),"standalone right correction finishes normally");
    ParkingRehearsal pausedPair(withCorrection);start(pausedPair,0);
    pausedPair.update(.7,RehearsalKey::Pause);pausedPair.holdCompleted(1,1,0);
    check(pausedPair.update(6,RehearsalKey::None)==5 && pausedPair.paused(),"recording pause does not block authorized automatic right correction");
    pausedPair.acknowledge(true,6);pausedPair.holdCompleted(1,1,1);
    check(pausedPair.finished() && pausedPair.paused(),"paired timer still finishes while recording is paused");
    ParkingRehearsal resumeAtSwitch(withCorrection);start(resumeAtSwitch,0);
    resumeAtSwitch.update(.7,RehearsalKey::Pause);resumeAtSwitch.holdCompleted(1,1,0);
    check(resumeAtSwitch.update(6,RehearsalKey::Continue)==5 && !resumeAtSwitch.paused(),"resume at automatic switch restores recording without losing the right command");
    ParkingRehearsal aborted(withCorrection);start(aborted,0);aborted.holdCompleted(1,1,0);
    check(!aborted.update(6,RehearsalKey::Quit) && aborted.finished(),"Q after left expiry prevents pending nonzero correction");
    ParkingRehearsal failedRight(withCorrection);start(failedRight,0);failedRight.holdCompleted(1,1,0);
    failedRight.update(6,RehearsalKey::None);failedRight.acknowledge(false,6);
    check(failedRight.outcome()=="SERVO_WRITE_FAILED","failed right write aborts pair");
    auto pairedFullTuning=withCorrection;pairedFullTuning.single=false;
    ParkingRehearsal pairedFull(pairedFullTuning);start(pairedFull,0);pairedFull.holdCompleted(1,1,0);
    pairedFull.update(6,RehearsalKey::None);pairedFull.acknowledge(true,6);pairedFull.holdCompleted(1,1,1);
    check(!pairedFull.finished(),"manual full mode records after complete paired bend");
    pairedFull.update(8,RehearsalKey::Enter);
    check(pairedFull.nextStage()==2,"completed pair advances to branch straight stage");
    check(pairedFull.update(9,RehearsalKey::Enter)==0 && pairedFull.stage()==2 && pairedFull.segment()==0,"next stage resets paired context");
    ParkingRehearsal savedPair(initial);savedPair.applySaved(withCorrection,0);savedPair.acknowledge(true,0);
    check(savedPair.automaticSavedTrial() && savedPair.trial()==1,"saving correction parameters starts paired stage trial");
    auto futureEdit=withCorrection;futureEdit.stages[3].command=11;savedPair.applySaved(futureEdit,1);
    savedPair.holdCompleted(1,1,0);savedPair.update(6,RehearsalKey::None);savedPair.acknowledge(true,6);
    auto rightEdit=futureEdit;rightEdit.stages[1].correction.command=7;
    check(savedPair.applySaved(rightEdit,6.1)==7 && savedPair.segment()==1 && savedPair.trial()==2,"editing active correction retunes right phase without replaying left");
    savedPair.acknowledge(true,6.1);savedPair.holdCompleted(1,1,1);
    check(!savedPair.finished(),"old right expiry cannot finish newly tuned correction");
    savedPair.holdCompleted(1,2,1);check(savedPair.finished(),"saved correction trial completes after its own timer");
    ParkingRehearsal disableRight(withCorrection);start(disableRight,0);disableRight.holdCompleted(1,1,0);
    disableRight.update(6,RehearsalKey::None);disableRight.acknowledge(true,6);
    auto disableEdit=withCorrection;disableEdit.stages[1].correction.holdSeconds=0;
    check(!disableRight.applySaved(disableEdit,6.1) && disableRight.outcome()=="RIGHT_CORRECTION_DISABLED","disabling active correction ends and centers instead of restarting left");
    SavedTuningWatcher correctionWatcher(initial,15);
    correctionWatcher.observe(correctionText,0);
    check(correctionWatcher.observe(correctionText,.4).has_value(),"watcher notices correction-only parameter changes");
    auto bothText=correctionText;
    bothText.insert(bothText.find("[stage_5]"),"correction_steer_command=-5\ncorrection_hold_time_s=1\n");
    auto both=ParkingTuning::parse(bothText);both.stage=4;both.stages[3].command=12;
    check(both.stages[1].correction.command==5 && both.stages[3].correction.command==-5,"two bends retain independent correction directions");
    check(initial.stages[3].correction.holdSeconds==0,"old config does not enable new stage4 correction");
    for(const auto& entry:{std::string("correction_steer_command=-5\n"),std::string("correction_hold_time_s=1\n")}) {
        auto partial=config();partial.insert(partial.find("[stage_5]"),entry);
        rejects([&] {ParkingTuning::parse(partial);},"stage4 correction must be specified as a complete pair");
    }
    for(double value:{0.,5.,-16.,std::numeric_limits<double>::quiet_NaN()}) {
        auto invalid=both;invalid.stages[3].correction.command=value;
        rejects([&] {invalid.validate();},"stage4 enabled correction must be a finite bounded left command");
    }
    for(double value:{-1.,121.,std::numeric_limits<double>::quiet_NaN()}) {
        auto invalid=both;invalid.stages[3].correction.holdSeconds=value;
        rejects([&] {invalid.validate();},"stage4 invalid correction duration rejected");
    }
    auto invalidSecond=both;invalidSecond.stages[3].command=-12;
    rejects([&] {invalidSecond.validate();},"stage4 primary must turn right when left correction enabled");
    invalidSecond=both;invalidSecond.stages[3].holdSeconds=0;
    rejects([&] {invalidSecond.validate();},"stage4 primary timer required for automatic correction");
    auto leftOnlyText=bothText;leftOnlyText.replace(leftOnlyText.find("stage=2"),7,"stage=4");
    leftOnlyText.insert(leftOnlyText.find("[stage_1]"),"stage_4_step=left_correction\n");
    const auto leftOnly=ParkingTuning::parse(leftOnlyText);
    check(leftOnly.stages[3].correctionOnly,"stage4 correction-only selector parsed");
    auto invalidOnly=leftOnly;invalidOnly.stage=2;
    rejects([&] {invalidOnly.validate();},"stage4 standalone selection requires stage4");
    invalidOnly=leftOnly;invalidOnly.single=false;
    rejects([&] {invalidOnly.validate();},"stage4 standalone selection requires single mode");
    invalidOnly=leftOnly;invalidOnly.stages[1].correctionOnly=true;
    rejects([&] {invalidOnly.validate();},"two standalone selectors cannot conflict");
    auto wrongOnlyText=leftOnlyText;wrongOnlyText.replace(wrongOnlyText.find("left_correction"),15,"right_correction");
    rejects([&] {ParkingTuning::parse(wrongOnlyText);},"stage4 rejects stage2 selector value");
    auto invalidNonBend=both;invalidNonBend.stages[2].correction={5,1};
    rejects([&] {invalidNonBend.validate();},"non-bend stage cannot execute a correction");
    ParkingRehearsal secondPair(both);start(secondPair,0);
    check(secondPair.command()==12 && std::string(secondPair.segmentName())=="SECOND_RIGHT_BEND","stage4 starts right bend with correct CSV label");
    secondPair.holdCompleted(3,1,0);
    check(secondPair.update(6,RehearsalKey::None)==-5 && !secondPair.finished(),"stage4 automatically changes to left correction");
    secondPair.acknowledge(true,6);
    check(std::string(secondPair.segmentName())=="LEFT_CORRECTION" && std::string(secondPair.segmentTitle())=="4B左打修正","stage4 correction labels are explicit");
    secondPair.holdCompleted(1,1,1);secondPair.holdCompleted(3,1,0);
    check(!secondPair.finished(),"wrong-stage and old primary events cannot complete left correction");
    secondPair.holdCompleted(3,1,1);check(secondPair.finished(),"stage4 ends after final correction timer");
    ParkingRehearsal standaloneLeft(leftOnly);start(standaloneLeft,0);
    check(standaloneLeft.command()==-5 && standaloneLeft.segment()==1,"standalone stage4 skips primary right bend");
    standaloneLeft.holdCompleted(3,1,1);check(standaloneLeft.finished(),"standalone left correction finishes");
    ParkingRehearsal pausedSecond(both);start(pausedSecond,0);
    pausedSecond.update(.7,RehearsalKey::Pause);pausedSecond.holdCompleted(3,1,0);
    check(pausedSecond.update(6,RehearsalKey::None)==-5 && pausedSecond.paused(),"stage4 automatically corrects left while recording paused");
    pausedSecond.acknowledge(true,6);pausedSecond.holdCompleted(3,1,1);
    check(pausedSecond.finished(),"paused stage4 finishes after correction");
    ParkingRehearsal abortSecond(both);start(abortSecond,0);abortSecond.holdCompleted(3,1,0);
    check(!abortSecond.update(6,RehearsalKey::Quit) && abortSecond.finished(),"Q prevents pending stage4 correction");
    ParkingRehearsal failSecond(both);start(failSecond,0);failSecond.holdCompleted(3,1,0);
    failSecond.update(6,RehearsalKey::None);failSecond.acknowledge(false,6);
    check(failSecond.finished(),"failed stage4 correction write aborts maneuver");
    ParkingRehearsal liveSecond(both);start(liveSecond,0);
    auto otherBend=both;otherBend.stages[1].correction.command=6;
    check(!liveSecond.applySaved(otherBend,1) && liveSecond.trial()==1,"editing first-bend correction does not restart second bend");
    liveSecond.holdCompleted(3,1,0);liveSecond.update(6,RehearsalKey::None);liveSecond.acknowledge(true,6);
    auto changedLeft=otherBend;changedLeft.stages[3].correction.command=-7;
    check(liveSecond.applySaved(changedLeft,6.1)==-7 && liveSecond.segment()==1 && liveSecond.trial()==2,"stage4 correction retunes without replaying right bend");
    liveSecond.acknowledge(true,6.1);liveSecond.holdCompleted(3,1,1);
    check(!liveSecond.finished(),"old stage4 correction timer does not finish retuned correction");
    auto changedPrimary=changedLeft;changedPrimary.stages[3].command=13;
    check(liveSecond.applySaved(changedPrimary,6.2)==13 && liveSecond.segment()==0,"primary edit restarts stage4 from right bend");
    liveSecond.acknowledge(true,6.2);liveSecond.holdCompleted(3,3,0);
    liveSecond.update(12,RehearsalKey::None);liveSecond.acknowledge(true,12);
    auto disableLeft=changedPrimary;disableLeft.stages[3].correction.holdSeconds=0;
    check(!liveSecond.applySaved(disableLeft,12.1) && liveSecond.outcome()=="LEFT_CORRECTION_DISABLED","disabling active left correction finishes rather than replaying primary");
    auto entire=both;entire.single=false;entire.stage=1;
    ParkingRehearsal entireFlow(entire);
    for(int index=0;index<6;++index) {
        const double now=20.*index;start(entireFlow,now);
        entireFlow.holdCompleted(index,entireFlow.trial(),0);
        if(isBendStage(index)) {
            check(entireFlow.update(now+6,RehearsalKey::None)==entire.stages[size_t(index)].correction.command,"full flow executes each bend correction");
            entireFlow.acknowledge(true,now+6);entireFlow.holdCompleted(index,entireFlow.trial(),1);
        }
        check(!entireFlow.finished() && entireFlow.stage()==index,"full flow continues recording after each complete maneuver");
        entireFlow.update(now+8,RehearsalKey::Enter);
        check(entireFlow.nextStage()==std::min(index+1,5),"full flow advances to next main stage");
    }
    SavedTuningWatcher bothWatcher(withCorrection,15);bothWatcher.observe(bothText,0);
    check(bothWatcher.observe(bothText,.4).has_value(),"watcher notices independent stage4 correction fields");
    const auto templateTuning=ParkingTuning::load("deploy/config/parking_tuning.ini");
    check(templateTuning.stages[1].correction.holdSeconds==0 && templateTuning.stages[3].correction.holdSeconds==0,"delivered template preserves successful baseline with extra corrections disabled");
    check(templateTuning.stage==1 && templateTuning.stages[0].motorLeft==2000 && templateTuning.stages[0].motorRight==2000 &&
        !templateTuning.stages[0].powered(),"template delivers requested 2000 commands with motion disabled until time configured");
    std::ostringstream actualSummary;writeTuningSummary(actualSummary,templateTuning,"/actual/config/parking_tuning.ini");
    check(actualSummary.str().find(std::string("VERSION ")+rehearsalVersion)!=std::string::npos && actualSummary.str().find("TUNING_FILE /actual/config/parking_tuning.ini")!=std::string::npos,"startup identifies the actual version and config path");
    check(actualSummary.str().find("右修正=关闭 5/0s")!=std::string::npos && actualSummary.str().find("左修正=关闭 -5/0s")!=std::string::npos,"startup identifies baseline disabled corrections and stored optional angles independently");
    std::ostringstream legacySummary;writeTuningSummary(legacySummary,initial,"legacy.ini");
    check(legacySummary.str().find("右修正=关闭")!=std::string::npos && legacySummary.str().find("左修正=关闭")!=std::string::npos,"legacy config clearly reports both corrections disabled");
    full=ParkingRehearsal(ParkingTuning::parse(config(false,6)));
    start(full,60.1);
    fullEdit.stage=6;
    check(full.applySaved(fullEdit,61).has_value() && full.stage()==5,"explicit selector edit reruns selected current stage");
    full.acknowledge(true,61);
    full.update(10000,RehearsalKey::None);check(!full.finished(),"stage6 does not auto finish recording");
    full.update(10001,RehearsalKey::Quit);check(full.outcome()=="USER_QUIT","manual finish");
    check(!full.update(10002,RehearsalKey::Enter),"quit latched");
    check(!full.applySaved(initial,10003),"Q dominates later file saves");
    ParkingRehearsal failed(initial);failed.update(0,RehearsalKey::Enter);failed.acknowledge(false,0);
    check(failed.outcome()=="SERVO_WRITE_FAILED" && !failed.update(1,RehearsalKey::Enter),"failed write latched");
    ParkingRehearsal clock(initial);clock.update(2,RehearsalKey::None);clock.update(1,RehearsalKey::Enter);
    check(clock.outcome()=="INVALID_TIME","time reversal prevents write");
    ParkingRehearsal nonfinite(initial);nonfinite.update(std::numeric_limits<double>::quiet_NaN(),RehearsalKey::Enter);
    check(nonfinite.finished(),"nonfinite time prevents write");
    for(char answer:{'Y','y'}) check(parseSaveChoice(answer)==SaveChoice::Yes,"Y accepts explicit retention");
    for(char answer:{'N','n'}) check(parseSaveChoice(answer)==SaveChoice::No,"N accepts explicit discard");
    for(char answer:{'\n','\r',' ','P','Q','0'}) check(parseSaveChoice(answer)==SaveChoice::Invalid,"save prompt ignores Enter and trial controls");
    for(char answer:{'Y','y','N','n'}) {
        std::ostringstream prompt;unsigned reads=0;
        const auto choice=readSaveConfirmation([&] {++reads;if(reads!=1) throw std::runtime_error("Waited for Enter");return answer;},prompt);
        check(choice==parseSaveChoice(answer) && reads==1,"single valid key completes immediately without Enter");
    }
    std::ostringstream prompt;const std::string input="\n PQ0\rN";size_t next=0;
    check(readSaveConfirmation([&] {return input.at(next++);},prompt)==SaveChoice::No,"invalid keys ignored until single N");
    const std::string label="【试验结束，采集已停止】";const auto shown=prompt.str();
    check(shown.find(label)==0 && shown.find(label,label.size())==std::string::npos,"save question appears once after multiple invalid keys");
    std::ostringstream failedPrompt;
    rejects([&] {readSaveConfirmation([]()->char {throw std::runtime_error("EOF");},failedPrompt);},"terminal failure cannot silently choose save or discard");
    namespace fs=std::filesystem;
    const auto buildRoot=fs::canonical("build");
    const auto root=buildRoot/("session_tests_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root);
    if(fs::canonical(root).parent_path()!=buildRoot) throw std::runtime_error("Test cleanup path outside build directory");
    try {
        const auto current=root/"rehearsal_current",older=root/"rehearsal_older";
        fs::create_directory(current);fs::create_directory(older);
        std::ofstream(older/"keep.txt")<<"previous data";
        SessionFiles files(current);std::ofstream(current/"01_advance.csv")<<"current data";
        files.discard();
        check(!fs::exists(current) && fs::exists(older/"keep.txt"),"N removes exclusively owned current session and preserves older data");
        fs::create_directory(current);SessionFiles kept(current);kept.saved();
        check(fs::exists(current) && !fs::exists(current/".pending_session"),"Y clears pending marker and preserves session");
        rejects([&] {kept.discard();},"saved session cannot later be discarded by stale owner");
        const auto tampered=root/"rehearsal_tampered";fs::create_directory(tampered);SessionFiles protectedFiles(tampered);
        std::ofstream(tampered/".pending_session",std::ios::trunc)<<"wrong owner";
        rejects([&] {protectedFiles.discard();},"changed ownership token blocks recursive discard");
        check(fs::exists(tampered),"rejected deletion preserves directory");
        const auto wrong=root/"config";fs::create_directory(wrong);
        rejects([&] {SessionFiles forbidden(wrong);},"configuration path cannot be owned as session");
        fs::remove_all(root);
    } catch(...) {fs::remove_all(root);throw;}
    std::cout<<checks<<" parking rehearsal checks passed\n";return 0;
 } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
