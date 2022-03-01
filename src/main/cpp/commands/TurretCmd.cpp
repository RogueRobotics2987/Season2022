// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurretCmd.h"

TurretCmd::TurretCmd(TurretSubsystem& l_turret, frc::Joystick& l_stick1, frc::Joystick& l_stick2, frc::Joystick& l_xbox) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_turret = &l_turret;
  m_stick1 = &l_stick1;
  m_stick2 = &l_stick2;
  m_xbox = &l_xbox;

  AddRequirements({m_turret});
}

// Called when the command is initially scheduled.
void TurretCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TurretCmd::Execute() {
  m_turret->setAngleV(m_xbox->GetRawAxis(1));
  m_turret->setAngleH(m_xbox->GetRawAxis(0));
  m_turret->setStickPOV(m_xbox->GetPOV());

}

// Called once the command ends or is interrupted.
void TurretCmd::End(bool interrupted) {}

// Returns true when the command should end.
bool TurretCmd::IsFinished() {
  return false;
}
