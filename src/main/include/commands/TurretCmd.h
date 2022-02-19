// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include "subsystems/TurretSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TurretCmd
    : public frc2::CommandHelper<frc2::CommandBase, TurretCmd> {
 public:
  TurretCmd(TurretSubsystem& l_turret, frc::Joystick& stick1, frc::Joystick& stick2, frc::Joystick& xbox);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
private:
  TurretSubsystem* m_turret = nullptr;
  frc::Joystick* m_stick1 = nullptr;
  frc::Joystick* m_stick2 = nullptr;
  frc::Joystick* m_xbox = nullptr;

};
