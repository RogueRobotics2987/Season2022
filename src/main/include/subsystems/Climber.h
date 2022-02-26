// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>


class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  void ClimbUp(double climbUpVal);
  void ClimbDown(double climbDownVal);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_climbMotor1{104, rev::CANSparkMax::MotorType::kBrushless}; 

};
