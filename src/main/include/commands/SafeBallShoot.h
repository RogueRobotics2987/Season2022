// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/TurretSubsystem.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include <frc/Timer.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SafeBallShoot
    : public frc2::CommandHelper<frc2::CommandBase, SafeBallShoot> {
 public:
  SafeBallShoot(Intake& l_intake, Shooter& l_shooter, TurretSubsystem& l_turret, double l_stopTime);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    Intake* m_intake = nullptr;
    Shooter* m_shooter = nullptr;
    TurretSubsystem* m_turret = nullptr;
    double stopTime = 20.0;
    frc::Timer m_timer;
    
};
