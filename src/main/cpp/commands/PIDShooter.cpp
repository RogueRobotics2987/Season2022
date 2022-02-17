// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PIDShooter.h"
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>


PIDShooter::PIDShooter() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void PIDShooter::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PIDShooter::Execute() {}

// Called once the command ends or is interrupted.
void PIDShooter::End(bool interrupted) {}

// Returns true when the command should end.
bool PIDShooter::IsFinished() {
  return false;
}
