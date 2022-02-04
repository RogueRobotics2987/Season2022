// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PickUpBall.h"

PickUpBall::PickUpBall(Intake* intake, frc::Joystick* xbox, frc::Joystick* stick2) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake = intake;
  m_xbox = xbox;
  m_stick2 = stick2;
}

// Called when the command is initially scheduled.
void PickUpBall::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PickUpBall::Execute() {
  //    if(m_xbox.GetRawButton(4)){
        
    //}
}

// Called once the command ends or is interrupted.
void PickUpBall::End(bool interrupted) {}

// Returns true when the command should end.
bool PickUpBall::IsFinished() {
  return false;
}
