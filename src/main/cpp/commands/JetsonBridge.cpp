// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/JetsonBridge.h"


JetsonBridge::JetsonBridge() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void JetsonBridge::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void JetsonBridge::Execute() {
  x = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumber("x",0.0);
  y = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumber("y",0.0);

  frc::SmartDashboard::PutNumber("DetectedBall X", x);
  frc::SmartDashboard::PutNumber("DetectedBall Y", y);
}
// Called once the command ends or is interrupted.
void JetsonBridge::End(bool interrupted) {}

// Returns true when the command should end.
bool JetsonBridge::IsFinished() {
  return false;
}
