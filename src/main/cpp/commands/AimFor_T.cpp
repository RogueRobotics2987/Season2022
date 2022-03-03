// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AimFor_T.h"

AimFor_T::AimFor_T(TurretSubsystem& l_turret, double l_time) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_time = l_time;
  m_turret.reset(&l_turret);
  AddRequirements({&l_turret});
}

// Called when the command is initially scheduled.
void AimFor_T::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  kp = -0.01f;
}

// Called repeatedly when this Command is scheduled to run
void AimFor_T::Execute() {
  float error = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  float adjustment = error * kp;
  m_turret->setSpeed(adjustment);
  }

// Called once the command ends or is interrupted.
void AimFor_T::End(bool interrupted) {  
  m_turret->setSpeed(0);
}

// Returns true when the command should end.
bool AimFor_T::IsFinished() {
  // if current time is < m_time, return false, else return true
  if(double(m_timer.Get()) < m_time) {
    return false;
  }  else {
    return true;
  }
  
}
