// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ServoTest.h"

ServoTest::ServoTest(Climber& climber, double l_time) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_time = l_time;
  m_climber = &climber;
  AddRequirements({m_climber});
}

// Called when the command is initially scheduled.
void ServoTest::Initialize() {
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void ServoTest::Execute() {
  
  if(double(m_timer.Get()) < m_time) {
    m_climber->ClimbServoLock();

  } else {
    m_climber->ClimbServoUnlock();
  }

}

// Called once the command ends or is interrupted.
void ServoTest::End(bool interrupted) {}

// Returns true when the command should end.
bool ServoTest::IsFinished() {
  return false;
}
