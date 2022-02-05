// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveTrain.h"


/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TankDrive
    : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  TankDrive(DriveTrain& drivetrain, frc::Joystick& stick1, frc::Joystick& stick2);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  //std::shared_ptr<frc::Joystick> m_stick1;
  //std::shared_ptr<frc::Joystick> m_stick2;
    frc::Joystick* m_stick1;
    frc::Joystick* m_stick2;
    DriveTrain* m_drivetrain;
  //std::shared_ptr<DriveTrain> m_drivetrain;
};
