// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <iostream>
#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>
#include "rev/CANPIDController.h"
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>


class Shooter : public frc2::SubsystemBase {
  public:
  Shooter();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void startShooter();
  void stopShooter();
  void setShooter();
  void setPercent(double percent); 
  double getVelocity(); 

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
 //shooterMotor = rev::CANSparkMax(47, rev::CANSparkMax::MotorType::kBrushless);
 // shooterPID = rev::CANPIDController(*shooterMotor);
  //shooterEncoder = rev::CANEncoder(*shooterMotor);
  rev::CANSparkMax shooterMotor= rev::CANSparkMax(10, rev::CANSparkMax::MotorType::kBrushless);
  rev::SparkMaxPIDController shooterPID= shooterMotor.GetPIDController();
  rev::SparkMaxRelativeEncoder shooterEncoder= shooterMotor.GetEncoder(); // might be wrong type of encoder
  double TargetRPM = 4000;
  double kp = 1E-4;
  double ki = 0; 
  double kd = 0;
  double kff = 2.05E-4; //old number 0.7/3500
  double mMaxV = 6000;
  double mMaxA = 2000;
  double mMinVelocityO = 500;
  double mCloseL = 100;
  const std::string firmwareVersion = "1.8.2"; 
  double Lastkp=0, Lastki=0, Lastkd=0, Lastkff=0;
  double arbFF = 0; 
};