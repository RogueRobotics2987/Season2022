// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PreAngles.h"

PreAngles::PreAngles(TurretSubsystem& l_turret, double l_TPosition) {
  // Use addRequirements() here to declare subsystem dependencies.

  m_turret = &l_turret; 
  position = l_TPosition; 
  
  AddRequirements({m_turret});

  SetName("Pre-Angles");

}

// Called when the command is initially scheduled.
void PreAngles::Initialize() {
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void PreAngles::Execute() {
  // turns to the approximate angle, off by -3 degrees

  // calculates position value in angles to motor turret value  
  calcPosition = position * 18/45;

  if(m_turret->getHPosition() > calcPosition) {
    m_turret->setAngleH(-0.6);

  } /*else if (position-5 < m_turret->getHPosition() && m_turret->getHPosition() < position+5 ){ // original
    m_turret->setAngleH(0.0);
  }*/ 
  else if (calcPosition-5 < m_turret->getHPosition() && m_turret->getHPosition() < calcPosition+5 /*m_turret->getHPosition() == calcPosition */) {
    m_turret->setAngleH(0.0); 

  } else {
    m_turret->setAngleH(0.6);

  }

  frc::SmartDashboard::PutNumber("preAngles test value", m_timer.Get().value());

}

// Called once the command ends or is interrupted.
void PreAngles::End(bool interrupted) {}

// Returns true when the command should end.
bool PreAngles::IsFinished() {

  //one or the other
  if(m_timer.Get().value() > 3.0) {
    return true;
  }

  /*if(position-5 < m_turret->getHPosition() && m_turret->getHPosition() < position+5){
    return true;
  }*/

  return false;
}
