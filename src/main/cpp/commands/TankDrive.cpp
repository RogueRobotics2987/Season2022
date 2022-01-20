// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(DriveTrain* drivetrain, frc::Joystick* stick1) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = drivetrain;
  m_stick1 = stick1;
  SetName("TankDrive");
  AddRequirements({m_drivetrain});
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  m_drivetrain->Drive(m_stick1->GetY(), m_stick1->GetX());
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {m_drivetrain->Drive(0,0);}

// Returns true when the command should end.
bool TankDrive::IsFinished() {
  return false;
}
