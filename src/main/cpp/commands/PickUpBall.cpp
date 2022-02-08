// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PickUpBall.h"

PickUpBall::PickUpBall(Intake& intake, frc::Joystick& xbox, frc::Joystick& stick2) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake = &intake;
  m_xbox = &xbox;
  m_stick2 = &stick2;
}

// Called when the command is initially scheduled.
void PickUpBall::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void PickUpBall::Execute() {
  /*if(m_xbox->GetRawButton(4)){//Intake brings ball in
    m_intake->IntakeBall(0.8);//80% of max speed
  }  else if(m_xbox->GetRawButton(2)) {//Intake brings ball out
    m_intake->IntakeBall(-0.8);//80% of max speed
  } else {
    m_intake->IntakeBall(0);//stops intake
  }*/

if (m_xbox->GetRawButton(4)){
  conveyorState = 1;
  intakeState = 1;
}

if(conveyorState == 1) {
 // frc2::InstantCommand([&m_intake] {m_conveyorForward.ConveyorForward(); },{m_intake});
}






}

// Called once the command ends or is interrupted.
void PickUpBall::End(bool interrupted) {}

// Returns true when the command should end.
bool PickUpBall::IsFinished() {
  return false;
}
