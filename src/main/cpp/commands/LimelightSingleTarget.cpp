// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LimelightSingleTarget.h"

LimelightSingleTarget::LimelightSingleTarget() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void LimelightSingleTarget::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LimelightSingleTarget::Execute() {
  tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
  frc::SmartDashboard::PutNumber("Limelight X", tx);
  frc::SmartDashboard::PutNumber("Limelight Y", ty);

  // std::cout << tx << std::endl;
  // std::cout << ty << std::endl;
}

// Called once the command ends or is interrupted.
void LimelightSingleTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool LimelightSingleTarget::IsFinished() {
  return false;
}
