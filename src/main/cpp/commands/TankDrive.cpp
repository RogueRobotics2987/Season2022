// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(DriveTrain* DriveTrain, frc :: Joystick*stick1) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = DriveTrain;
  m_stick1 = stick1;
  SetName("TankDrive");
  AddRequirements({m_drivetrain});
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {

  double outputY;
  double outputX;

  if(m_stick1->GetY() > 0.5 ){
  outputY = 0.5;
  }  else {
    outputY = m_stick1->GetY();
  }

  if(m_stick1->GetX() > 0.5){
    outputX = 0.5;
  } else {
  outputX = m_stick1->GetX();
  }
  m_drivetrain->Drive(outputY, outputX);
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {m_drivetrain->Drive(0, 0);}

// Returns true when the command should end.
bool TankDrive::IsFinished() {
  return false;
}
