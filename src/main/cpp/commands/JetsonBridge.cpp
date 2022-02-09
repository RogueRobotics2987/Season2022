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
  std::vector< double > xArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumberArray("Left X", defaultValReturn);
  std::vector< double > yArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumberArray("Top Y",defaultValReturn);
  std::vector< std::string > labelArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetStringArray("Label",defaultStringReturn);
  std::vector< double > areaArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumberArray("Area",defaultValReturn);
  std::vector< double > confArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumberArray("Confidence",defaultValReturn);

  // frc::SmartDashboard::PutNumber("DetectedBall X", x);
  // frc::SmartDashboard::PutNumber("DetectedBall Y", y);
}
// Called once the command ends or is interrupted.
void JetsonBridge::End(bool interrupted) {}

// Returns true when the command should end.
bool JetsonBridge::IsFinished() {
  return false;
}
