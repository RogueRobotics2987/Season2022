// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/StartUpSequence.h"

StartUpSequence::StartUpSequence() {
  // Use addRequirements() here to declare subsystem dependencies.
  myTimer.Reset();
  myTimer.Start();


}

// Called when the command is initially scheduled.
void StartUpSequence::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void StartUpSequence::Execute() {
  units::time::second_t startTime = myTimer.Get();
/*The super wacky start up sequence sure to make us die of laughter
the 1 foot tall super spinny turret starts facing intake
creepy climber that looks weird  moves forward towards intake pushing it down towards the center of the earth
ball grabing intake drops down
super speedy turret spins 180 counterclockwise? unless it turns into helleboper




*/
}

// Called once the command ends or is interrupted.
void StartUpSequence::End(bool interrupted) {}

// Returns true when the command should end.
bool StartUpSequence::IsFinished() {
  return false;
} 

