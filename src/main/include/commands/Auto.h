// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveTrain.h"
#include <units/voltage.h>
#include <frc/timer.h>
#include <iostream>


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class Auto
    : public frc2::CommandHelper<frc2::CommandBase, Auto> {
 public:
  Auto(DriveTrain& l_drivetrain, double l_time, double l_volts);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
    DriveTrain* m_drivetrain = nullptr;
    double m_time = 0.0;
    frc::Timer m_timer;
    double m_volts = 0.0;
};
