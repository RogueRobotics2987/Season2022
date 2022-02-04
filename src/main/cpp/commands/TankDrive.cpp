// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(DriveTrain& drivetrain, frc::Joystick& stick1, frc::Joystick& stick2) {
  // Use addRequirements() here to declare subsystem dependencies.
  //m_drivetrain.reset(std::make_shared<DriveTrain>(drivetrain));
    m_drivetrain.reset(&drivetrain);
    m_stick1.reset(&stick1);
    m_stick2.reset(&stick2);
  SetName("TankDrive");
  AddRequirements({&drivetrain});
  frc::SmartDashboard::PutBoolean("Acceleration Control", false); 
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
m_drivetrain->Reset();//works if lowercase

}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
 // m_drivetrain->Drive(m_stick1->GetY(), m_stick1->GetX()); //simple drive code

  static double lastTurnVal = 0.0; 
  static double lastSpeedVal = 0.0;//speed means drive forward or backwards

  double stickTurnVal = m_stick1->GetX(); //getting the Y value from the joystick
  double stickSpeedVal = m_stick2->GetY(); //comment
  double outTurnVal = 0;
  double outSpeedVal = 0;
  double maxChange = 0.04; //per second
 
 //frc::SmartDashboard::PutNumber("lastLeft Value", lastLeft);
 //frc::SmartDashboard::PutNumber("Left value", Left);
 //frc::SmartDashboard::PutNumber("lastRight Value", lastRight);
 //frc::SmartDashboard::PutNumber("Right value", Right);
 frc::SmartDashboard::GetNumber("maxChange", maxChange); 
 maxChange = frc::SmartDashboard::GetNumber("maxChange", maxChange); 

 bool accelCtrl = false; 
 accelCtrl = frc::SmartDashboard::GetBoolean("Acceleration Control", false); 

  if (abs(stickTurnVal-lastTurnVal) >maxChange && accelCtrl) {
    outTurnVal = lastTurnVal + copysignf(1.0,stickTurnVal - lastTurnVal)*maxChange;
    } else {
      outTurnVal = stickTurnVal;
  }
  if (abs(stickSpeedVal-lastSpeedVal) >maxChange && accelCtrl) {
    outSpeedVal = lastSpeedVal + copysignf(1.0,stickSpeedVal - lastSpeedVal)*maxChange;
    } else {
      outSpeedVal = stickSpeedVal;
  }
  
  m_drivetrain->Drive(outSpeedVal, outTurnVal);
   lastTurnVal = outTurnVal;
  lastSpeedVal = outSpeedVal; }

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {m_drivetrain->Drive(0,0);}

// Returns true when the command should end.
bool TankDrive::IsFinished() {
  return false;
}
