// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TrimAngle.h"

TrimAngle::TrimAngle(frc::Joystick* xbox, ShooterActuator* Actuator, frc::Joystick* stick2) {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void TrimAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TrimAngle::Execute() {}

// Called once the command ends or is interrupted.
void TrimAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool TrimAngle::IsFinished() {
  return false;
}
