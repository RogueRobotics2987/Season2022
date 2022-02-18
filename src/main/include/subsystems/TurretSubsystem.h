// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h" 
#include <frc/Joystick.h>

class TurretSubsystem : public frc2::SubsystemBase {
 public:
  TurretSubsystem();
  void setSpeed(float speed);
  void setAngleH();
  void setAngleV();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

 frc::Joystick* m_xBox = nullptr;

  // rev::CANSparkMax m_turret = rev::CANSparkMax(60, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax m_VerturretMotor{14, rev::CANSparkMax::MotorType::kBrushless}; 

  int actuatorState;

  rev::SparkMaxLimitSwitch ls_turret = m_VerturretMotor.GetForwardLimitSwitch(
                             rev::SparkMaxLimitSwitch::LimitSwitchPolarity::kNormallyClosed);

  


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
