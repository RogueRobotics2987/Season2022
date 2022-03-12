// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurretSetAngleCmd.h"

TurretSetAngleCmd::TurretSetAngleCmd(TurretSubsystem& l_turret, double vPosition, double hPosition) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_turret = &l_turret; 
  m_vPosition = vPosition; 
  m_hPosition = hPosition;
  
  AddRequirements({m_turret});

}

// Called when the command is initially scheduled.
void TurretSetAngleCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TurretSetAngleCmd::Execute() {
  if(m_turret->getHPosition() > m_hPosition) {
    m_turret->setAngleH(-0.6);

  } else if (m_hPosition-5 < m_turret->getHPosition() && m_turret->getHPosition() < m_hPosition+5) {
    m_turret->setAngleH(0.0);

  } else {
    m_turret->setAngleH(0.6);

  }

  if(m_turret->getVPosition() > m_vPosition) {
    m_turret->setAngleH(-0.6);

  } else if (m_vPosition-5 < m_turret->getVPosition() && m_turret->getVPosition() < m_vPosition+5) {
    m_turret->setAngleV(0.0);

  } else {
    m_turret->setAngleV(0.6);

  }

  //frc::SmartDashboard::PutNumber("preAngles test value", m_timer.Get().value());

}

// Called once the command ends or is interrupted.
void TurretSetAngleCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool TurretSetAngleCmd::IsFinished() {
  
  if(m_hPosition-5 < m_turret->getHPosition() && m_turret->getHPosition() < m_hPosition+5){
    return true;
  }

  return false;
}
