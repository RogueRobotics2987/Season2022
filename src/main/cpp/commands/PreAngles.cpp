// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PreAngles.h"

PreAngles::PreAngles(TurretSubsystem& l_turret, double l_TPosition) {
  // Use addRequirements() here to declare subsystem dependencies.

  m_turret = &l_turret; 
  position = l_TPosition; 

}

// Called when the command is initially scheduled.
void PreAngles::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PreAngles::Execute() {
  if(m_turret->getHPosition() > position) {
    m_turret->setAngleH(-0.2);

  } else if (position-5 < m_turret->getHPosition() && m_turret->getHPosition() < position+5) {
    m_turret->setAngleH(0.0);

  } else {
    m_turret->setAngleH(0.2);

  }
}

// Called once the command ends or is interrupted.
void PreAngles::End(bool interrupted) {}

// Returns true when the command should end.
bool PreAngles::IsFinished() {

  if(0.0 > 3.0) {
    return true;
  }
  return false;
}
