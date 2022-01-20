// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h" 
#include <frc/drive/DifferentialDrive.h>


class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  void Drive(double y, double z);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax LeftBack = rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless);//56 on Howie
  rev::CANSparkMax LeftFront = rev::CANSparkMax(1, rev::CANSparkMax::MotorType::kBrushless);//49 on Howie
  rev::CANSparkMax RightBack = rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless);//50 on Howie
  rev::CANSparkMax RightFront = rev::CANSparkMax(4, rev::CANSparkMax::MotorType::kBrushless);//46 on Howie
  frc::DifferentialDrive m_robotDrive{LeftFront, RightFront};
};
