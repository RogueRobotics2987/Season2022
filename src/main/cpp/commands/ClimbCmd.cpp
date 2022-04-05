// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbCmd.h"

ClimbCmd::ClimbCmd(Climber& climber, frc::Joystick& xbox, frc::Joystick& stick1, frc::Joystick& stick2) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_xbox = &xbox;
  m_stick1 = &stick1;
  m_stick2 = &stick2;
  m_climber = &climber;
  AddRequirements({m_climber});

}

// Called when the command is initially scheduled.
void ClimbCmd::Initialize() {
  matchTimer.Reset();
  matchTimer.Start();
}

// Called repeatedly when this Command is scheduled to run
void ClimbCmd::Execute() {
  //Axis 3 is right trigger
  //Axis 2 is left trigger
  m_climber->ClimbFunction(m_xbox->GetRawAxis(3), m_xbox->GetRawAxis(2));

  if(150 - double(matchTimer.Get()) < .5){
    m_climber->ClimbServoLock(); 
  }
}

// Called once the command ends or is interrupted.
void ClimbCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimbCmd::IsFinished() {
  return false;
}
