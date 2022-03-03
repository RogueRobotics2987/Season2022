// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Auto.h"

Auto::Auto(DriveTrain& l_drivetrain, double l_time, double l_percent) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = &l_drivetrain;
  m_time = l_time;
  m_percent = -l_percent;
  AddRequirements({m_drivetrain});
}

// Called when the command is initially scheduled.
void Auto::Initialize() {
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void Auto::Execute() {
  m_drivetrain->Drive(m_percent, 0);
  //std::cout<<"cur_time: " << m_timer.Get().value() << "volts: " << m_volts << std::endl;
}

// Called once the command ends or is interrupted.
void Auto::End(bool interrupted) {
  m_drivetrain->Drive(0, 0);
  m_drivetrain = nullptr;
}

// Returns true when the command should end.
bool Auto::IsFinished() {
  // if current time is < m_time, return false, else return true
  if(double(m_timer.Get()) < m_time) {
    return false;
  }  else {
    return true;
  }
  
}
