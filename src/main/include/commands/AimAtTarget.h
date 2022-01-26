// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/AutoAim.h"
#include "subsystems/TurretSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AimAtTarget
    : public frc2::CommandHelper<frc2::CommandBase, AimAtTarget> {
 public:



  AimAtTarget();

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  float kp;
  TurretSubsystem m_turret;

};
