// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include <frc2/command/SubsystemBase.h>

class intake : public frc2::SubsystemBase {
 public:
  intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void StartConveyor (double percent);
  void intakeBall (double percent);
  void StopMotors ();

   private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax* p_intakeMotor = new rev::CANSparkMax (51, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax* p_conveyorMotor = new rev::CANSparkMax (48, rev::CANSparkMax::MotorType::kBrushless);
};
