// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Pickupball.h"

Pickupball::Pickupball(intake* intake, frc::Joystick *joy1) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake = intake;
  m_joy1 = joy1;
  SetName ("Pickupball") ;
  AddRequirements({m_intake});
  
  }

// Called when the command is initially scheduled.
void Pickupball::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Pickupball::Execute() {
  if(m_joy1->GetRawButton (5)) {
    m_intake->intakeBall (.65); // intake in
  }
  else if (m_joy1->GetRawButton (3)) {
    m_intake->intakeBall (-.65);
  }
  else{
    m_intake->intakeBall (0);
  }
  if(m_joy1->GetRawButton (4)) {
    m_intake->StartConveyor (.5) ;
  }
  else if (m_joy1->GetRawButton (6)) {
    m_intake->StartConveyor (-.3);
  }
  else{
    m_intake->StartConveyor(0);
  }
}

// Called once the command ends or is interrupted.
void Pickupball::End(bool interrupted) {
  m_intake->StopMotors();
}

// Returns true when the command should end.
bool Pickupball::IsFinished() {
  return false;
}
