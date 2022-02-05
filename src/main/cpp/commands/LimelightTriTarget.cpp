// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LimelightTriTarget.h"

LimelightTriTarget::LimelightTriTarget() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void LimelightTriTarget::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void LimelightTriTarget::Execute() {
  tx0 = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx0",0.0);
  ty0 = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty0",0.0);
  tx1 = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx1",0.0);
  ty1 = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty1",0.0);
  tx2 = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx2",0.0);
  ty2 = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty2",0.0);
  frc::SmartDashboard::PutNumber("Limelight X0", tx0);
  frc::SmartDashboard::PutNumber("Limelight Y0", ty0);
  frc::SmartDashboard::PutNumber("Limelight X1", tx1);
  frc::SmartDashboard::PutNumber("Limelight Y1", ty1);
  frc::SmartDashboard::PutNumber("Limelight X2", tx2);
  frc::SmartDashboard::PutNumber("Limelight Y2", ty2);

  std::cout << tx0 << std::endl;
  std::cout << ty0 << std::endl;
  std::cout << tx1 << std::endl;
  std::cout << ty1 << std::endl;
  std::cout << tx2 << std::endl;
  std::cout << ty2 << std::endl;
}

// Called once the command ends or is interrupted.
void LimelightTriTarget::End(bool interrupted) {}

// Returns true when the command should end.
bool LimelightTriTarget::IsFinished() {
  return false;
}
