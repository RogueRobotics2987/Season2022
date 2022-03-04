// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
//#include <frc/DigitalInput.h>


class Climber : public frc2::SubsystemBase {
 public:
  Climber();
  void ClimbFunction(double climbUpVal, double climbDownVal);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_climbMotorRight{8, rev::CANSparkMax::MotorType::kBrushless}; //#8 on climber
  rev::CANSparkMax m_climbMotorLeft{9, rev::CANSparkMax::MotorType::kBrushless}; //#9 on climber
  //frc::DigitalInput ls_climbRight{0};
  //frc::DigitalInput ls_climbLeft{1};

  rev::SparkMaxRelativeEncoder re_climbMotorRight = m_climbMotorRight.GetEncoder();
  rev::SparkMaxRelativeEncoder re_climbMotorLeft = m_climbMotorLeft.GetEncoder();

  double climbVal = 0.0;
  double upDeadzone = 0.08;
  double downDeadzone = 0.08;
  double RC = 0.0;
  double LC = 0.0;
  double error = 0.0;
  double climbKValue = 0.0;
  

};
