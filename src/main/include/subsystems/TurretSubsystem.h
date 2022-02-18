// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h" 
#include <iostream>


class TurretSubsystem : public frc2::SubsystemBase {
 public:
  TurretSubsystem();
  void setSpeed(float speed);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // rev::CANSparkMax m_vertMotor{47, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_vertMotor = rev::CANSparkMax(47, rev::CANSparkMax::MotorType::kBrushless);

      rev::SparkMaxLimitSwitch ls_vertMotor = m_vertMotor.GetForwardLimitSwitch(
                             rev::SparkMaxLimitSwitch::LimitSwitchPolarity::kNormallyClosed);
  
    // bool ls_vertMotor = false;
  int state = 0;

  double ActuatorSpeed = 0.0;


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
