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
    model.acknowledge(true,time);check(!model.update(time+.49,RehearsalKey::Enter),"settling discards Enter");
    model.update(time+.5,RehearsalKey::Enter);
    check(model.state()==ParkingRehearsal::State::AwaitStart,"settling boundary never starts trial");
    check(!model.update(time+.51,RehearsalKey::Enter),"start permission does not write");
    check(model.state()==ParkingRehearsal::State::Running,"second permission starts");
}
int main() {
 try {
    const auto initial=ParkingTuning::parse(config());check(initial.stage==2 && initial.single,"single stage2 parsed");
    check(ParkingTuning::parse("\xef\xbb\xbf"+config()).stage==2,"UTF8 BOM accepted");
    check(initial.correction.holdSeconds==0,"older tuning files do not silently enable a new nonzero command");
    auto narrowLegacy=initial;for(auto& stage:narrowLegacy.stages) stage.command=0;
    narrowLegacy.validate(4);
    check(narrowLegacy.correction.command==0,"disabled legacy correction does not introduce a command above a narrow vehicle limit");
    auto correctionText=config();correctionText.insert(correctionText.find("[stage_3]"),"correction_steer_command=5\ncorrection_hold_time_s=1\n");
    const auto withCorrection=ParkingTuning::parse(correctionText);
    check(withCorrection.correction.command==5 && withCorrection.correction.holdSeconds==1,"right correction parameters parsed");
    for(const auto& entry:{std::string("correction_steer_command=5\n"),std::string("correction_hold_time_s=1\n")}) {
        auto partial=config();partial.insert(partial.find("[stage_3]"),entry);
        rejects([&] {ParkingTuning::parse(partial);},"partially specified correction rejected");
    }
    auto unsafeCorrection=withCorrection;unsafeCorrection.correction.command=-5;
    rejects([&] {unsafeCorrection.validate();},"left correction forbidden for this right correction action");
    unsafeCorrection=withCorrection;unsafeCorrection.stages[1].command=5;
    rejects([&] {unsafeCorrection.validate();},"first bend must remain opposite to correction");
    unsafeCorrection=withCorrection;unsafeCorrection.stages[1].holdSeconds=0;
    rejects([&] {unsafeCorrection.validate();},"enabled correction requires timed first bend");
    auto missingCore=correctionText;missingCore.erase(missingCore.find("mode=single\n"),12);
    rejects([&] {ParkingTuning::parse(missingCore);},"optional keys cannot replace a required core key");
    auto correctionOnlyText=correctionText;correctionOnlyText.insert(correctionOnlyText.find("[stage_1]"),"stage_2_step=right_correction\n");
    const auto correctionOnly=ParkingTuning::parse(correctionOnlyText);
    check(correctionOnly.startRightCorrection,"correction-only selection parsed");
    auto invalidSelection=correctionOnly;invalidSelection.single=false;
    rejects([&] {invalidSelection.validate();},"correction-only cannot skip main bend in full mode");
    invalidSelection=correctionOnly;invalidSelection.stage=4;
    rejects([&] {invalidSelection.validate();},"correction-only is limited to stage2");
    invalidSelection=correctionOnly;invalidSelection.correction.holdSeconds=0;
    rejects([&] {invalidSelection.validate();},"correction-only requires positive hold time");
    auto disabled=withCorrection;disabled.correction.holdSeconds=0;disabled.validate();
    check(disabled.correction.holdSeconds==0,"zero correction duration disables automatic extra command");
    for(double value:{-1.,121.,std::numeric_limits<double>::quiet_NaN()}) {
        auto invalid=withCorrection;invalid.correction.holdSeconds=value;
        rejects([&] {invalid.validate();},"invalid correction duration rejected");
    }
    for(double value:{0.,16.,std::numeric_limits<double>::quiet_NaN()}) {
        auto invalid=withCorrection;invalid.correction.command=value;
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
    auto rightEdit=futureEdit;rightEdit.correction.command=7;
    check(savedPair.applySaved(rightEdit,6.1)==7 && savedPair.segment()==1 && savedPair.trial()==2,"editing active correction retunes right phase without replaying left");
    savedPair.acknowledge(true,6.1);savedPair.holdCompleted(1,1,1);
    check(!savedPair.finished(),"old right expiry cannot finish newly tuned correction");
    savedPair.holdCompleted(1,2,1);check(savedPair.finished(),"saved correction trial completes after its own timer");
    ParkingRehearsal disableRight(withCorrection);start(disableRight,0);disableRight.holdCompleted(1,1,0);
    disableRight.update(6,RehearsalKey::None);disableRight.acknowledge(true,6);
    auto disableEdit=withCorrection;disableEdit.correction.holdSeconds=0;
    check(!disableRight.applySaved(disableEdit,6.1) && disableRight.outcome()=="RIGHT_CORRECTION_DISABLED","disabling active correction ends and centers instead of restarting left");
    SavedTuningWatcher correctionWatcher(initial,15);
    correctionWatcher.observe(correctionText,0);
    check(correctionWatcher.observe(correctionText,.4).has_value(),"watcher notices correction-only parameter changes");
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
