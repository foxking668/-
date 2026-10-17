#pragma once
#include <string>

namespace Contral {class Motor;class Steer;}
namespace AutoParking {
void initialize(const std::string& configPath);
bool enabled();
bool takeoverActive();
bool cameraNeeded();
void zebraDetected(int count);
void publishLane(bool valid,double error);
// Image and motor threads share this gate; no late image write can undo a reverse command.
bool writeCruiseSteering(Contral::Steer* steer,double error);
bool motorStep(Contral::Motor* motor,Contral::Steer* steer,double leftRps,double rightRps,bool zebraHeld);
void cameraReleased();
void shutdown();
}
