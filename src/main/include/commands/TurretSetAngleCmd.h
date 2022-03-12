// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/TurretSubsystem.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TurretSetAngleCmd
    : public frc2::CommandHelper<frc2::CommandBase, TurretSetAngleCmd> {
 public:
  TurretSetAngleCmd(TurretSubsystem& l_turret, double vPosition, double hPosition);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
  
  private: 
    TurretSubsystem* m_turret = nullptr;
    double m_hPosition = 0.0;
    double m_vPosition = 0.0;

};
