// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TankDrive.h"

TankDrive::TankDrive(DriveTrain& drivetrain, frc::Joystick& stick1, frc::Joystick& stick2) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_drivetrain = &drivetrain;
  m_stick1 = &stick1;
  m_stick2 = &stick2;
  SetName("TankDrive");
  AddRequirements({m_drivetrain});
  frc::SmartDashboard::PutBoolean("Acceleration Control", false); 
  frc::SmartDashboard::PutBoolean("AccelCtrlWorking", false);
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
m_drivetrain->Reset();

}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
 // m_drivetrain->Drive(m_stick1->GetY(), m_stick1->GetX()); //simple drive code

  static double lastTurnVal = 0.0; 
  static double lastSpeedVal = 0.0;//speed means drive forward or backwards

  double stickTurnVal = m_stick1 -> GetX(); //getting the Y value from the joystick
  double stickSpeedVal = m_stick2 -> GetY(); //comment
  double outTurnVal = 0;
  double outSpeedVal = 0;
  double maxChange = 0.06; //per unit of time //was 0.04
 
 frc::SmartDashboard::PutNumber("lastTurnVal", lastTurnVal);
 frc::SmartDashboard::PutNumber("stickTurnVal", stickTurnVal);
 frc::SmartDashboard::PutNumber("lastSpeedVal", lastSpeedVal);
 frc::SmartDashboard::PutNumber("stickSpeedVal", stickSpeedVal);
 frc::SmartDashboard::GetNumber("maxChange", maxChange); 
 maxChange = frc::SmartDashboard::GetNumber("maxChange", maxChange); 

 bool accelCtrl = false; 
 accelCtrl = frc::SmartDashboard::GetBoolean("Acceleration Control", false); 

 if (accelCtrl == true){
   frc::SmartDashboard::PutBoolean("AccelCtrlWorking", true);
 } else {
   frc::SmartDashboard::PutBoolean("AccelCtrlWorking", false);
 }

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
  
  m_drivetrain -> Drive(outSpeedVal, outTurnVal);
   lastTurnVal = outTurnVal;
  lastSpeedVal = outSpeedVal;
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {m_drivetrain->Drive(0,0);}

// Returns true when the command should end.
bool TankDrive::IsFinished() {
  return false;
}
