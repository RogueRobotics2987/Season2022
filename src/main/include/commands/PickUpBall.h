// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/Intake.h"
#include <frc/Joystick.h>
#include <frc2/command/InstantCommand.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PickUpBall
    : public frc2::CommandHelper<frc2::CommandBase, PickUpBall> {
 public:
  PickUpBall(Intake& intake, frc::Joystick& xbox, frc::Joystick& stick2);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  private:
  Intake* m_intake = nullptr;
  frc::Joystick* m_xbox = nullptr;
  frc::Joystick* m_stick2 = nullptr;
  int conveyorState = 0;
  int intakeState = 0;

};
