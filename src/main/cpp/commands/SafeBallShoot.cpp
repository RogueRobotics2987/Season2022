// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SafeBallShoot.h"

SafeBallShoot::SafeBallShoot(TurretSubsystem& l_turret, double l_stopTime) {
  // Use addRequirements() here to declare subsystem dependencies.
  // m_intake = &l_intake;
  // m_shooter = &l_shooter;
  m_turret = &l_turret;
  stopTime = l_stopTime;
  // AddRequirements({m_intake});
  // AddRequirements({m_shooter});
  AddRequirements({m_turret});
}

// Called when the command is initially scheduled.
void SafeBallShoot::Initialize() {
  m_timer.Reset();
  m_timer.Start();
  m_turret->setAutoAimOn();
  // m_shooter->setShooter();

}

// Called repeatedly when this Command is scheduled to run
void SafeBallShoot::Execute() {
  // if turret speed > 3700 & tx < 1, ty < 1 then conveyor forward
  float tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
  float ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
  float tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv",0.0);
  // frc::SmartDashboard::PutNumber("Shooter RPM", m_shooter->getVelocity());

  if(-1.0 < tx && tx < 1.0 && -2 < ty && ty < 2 && tv == 1) {//ty was 2.5
    // m_intake->ConveyorForward();
    frc::SmartDashboard::PutBoolean("Target Locked", true);
    m_LockedOn = true;
  } else {
  frc::SmartDashboard::PutBoolean("Target Locked", false);
    m_LockedOn = false;
  }
}

// Called once the command ends or is interrupted.
void SafeBallShoot::End(bool interrupted) {
  m_turret->setManualAimOn();
  // m_intake->ConveyorForwardRelease();
}

// Returns true when the command should end.
bool SafeBallShoot::IsFinished() {
  if(m_timer.Get().value() > stopTime || m_LockedOn == true) {
    return true;
  } else {
    return false;
  }
}
