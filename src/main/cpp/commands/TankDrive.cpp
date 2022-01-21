// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(DriveTrain* drivetrain, frc::Joystick* stick1, frc::Joystick* stick2) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = drivetrain;
  m_stick1 = stick1;
  m_stick2 = stick2;
  SetName("TankDrive");
  AddRequirements({m_drivetrain});
  frc::SmartDashboard::PutBoolean("Acceleration Control", false); 
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
m_drivetrain->Reset();

}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
 // m_drivetrain->Drive(m_stick1->GetY(), m_stick1->GetX()); //simple drive code

static double lastLeft = 0.0; 
  static double lastRight = 0.0;

  double Left = m_stick1 -> GetX(); //getting the Y value from the joystick
  double Right = m_stick2 -> GetY(); //comment
  double outLeft = 0;
  double outRight = 0;
  double maxChange = 0.04; //per second
 
 //frc::SmartDashboard::PutNumber("lastLeft Value", lastLeft);
 //frc::SmartDashboard::PutNumber("Left value", Left);
 //frc::SmartDashboard::PutNumber("lastRight Value", lastRight);
 //frc::SmartDashboard::PutNumber("Right value", Right);
 frc::SmartDashboard::GetNumber("maxChange", maxChange); 
 maxChange = frc::SmartDashboard::GetNumber("maxChange", maxChange); 

 bool accelCtrl = false; 
 accelCtrl = frc::SmartDashboard::GetBoolean("Acceleration Control", false); 

  if (abs(Left-lastLeft) >maxChange && accelCtrl) {
    outLeft = lastLeft + copysignf(1.0,Left - lastLeft)*maxChange;
    } else {
      outLeft = Left;
  }
  if (abs(Right-lastRight) >maxChange && accelCtrl) {
    outRight = lastRight + copysignf(1.0,Right - lastRight)*maxChange;
    } else {
      outRight = Right;
  }
  
  m_drivetrain -> Drive(outLeft, outRight);
   lastLeft = outLeft;
  lastRight = outRight; }

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {m_drivetrain->Drive(0,0);}

// Returns true when the command should end.
bool TankDrive::IsFinished() {
  return false;
}
