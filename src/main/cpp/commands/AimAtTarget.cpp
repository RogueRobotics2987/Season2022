// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AimAtTarget.h"

AimAtTarget::AimAtTarget(TurretSubsystem& turret) {
  // Use addRequirements() here to declare subsystem dependencies.
  // m_turret = turret;
  m_turret.reset(&turret);
}

// Called when the command is initially scheduled.
void AimAtTarget::Initialize() {
 kp = -0.1f;
}

// Called repeatedly when this Command is scheduled to run
void AimAtTarget::Execute() {
  float error = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  float adjustment = error * kp;
  m_turret->setSpeed(adjustment);
}

// Called once the command ends or is interrupted.
void AimAtTarget::End(bool interrupted) {
  m_turret->setSpeed(0);
}

// Returns true when the command should end.
bool AimAtTarget::IsFinished() {
  return false;
}
