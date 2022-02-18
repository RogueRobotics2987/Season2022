// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h" 
#include <frc/Joystick.h>
#include <iostream>

class TurretSubsystem : public frc2::SubsystemBase {
 public:
  TurretSubsystem();
  void setSpeed(float speed);
  void setAngleH();
  void setAngleV(float l_stickValV);
  void setAngleH(float l_stickValH);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

   frc::Joystick* m_xBox = nullptr;

  // rev::CANSparkMax m_turret = rev::CANSparkMax(60, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax m_vTurretMotor{13, rev::CANSparkMax::MotorType::kBrushless}; 
  rev::CANSparkMax m_hTurretMotor{14, rev::CANSparkMax::MotorType::kBrushless}; 
  // rev::SparkMaxLimitSwitch ls_turret = m_vTurretMotor.GetForwardLimitSwitch(
  //                            rev::SparkMaxLimitSwitch::LimitSwitchPolarity::kNormallyClosed);

  int actuatorState;
  double cur_stickValV = 0.0;
  double cur_stickValH = 0.0;

  


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
