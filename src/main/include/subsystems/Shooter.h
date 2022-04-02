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
  //void startShooter();
  void stopShooter();
  void reverseShooter();
  void setShooter();
  void setPercent(double percent); 
  double getVelocity(); 

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
 //shooterMotor = rev::CANSparkMax(47, rev::CANSparkMax::MotorType::kBrushless);
 // shooterPID = rev::CANPIDController(*shooterMotor);
  //shooterEncoder = rev::CANEncoder(*shooterMotor);
  rev::CANSparkMax shooterMotorFront= rev::CANSparkMax(10, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax shooterMotorBack= rev::CANSparkMax(11, rev::CANSparkMax::MotorType::kBrushless);
  rev::SparkMaxPIDController FshooterPID= shooterMotorFront.GetPIDController();
  rev::SparkMaxPIDController BshooterPID= shooterMotorBack.GetPIDController();

  rev::SparkMaxRelativeEncoder shooterEncoderFront= shooterMotorFront.GetEncoder(); 
  rev::SparkMaxRelativeEncoder shooterEncoderBack= shooterMotorBack.GetEncoder(); 
  double FTargetRPM = 2950;
  double BTargetRPM = 2950;
  double Fkp = 1E-4; //F is for front
  double Fki = 0; 
  double Fkd = 0;
  double Fkff = 2.05E-4; //old number 0.7/3500
  double FmMaxV = 6000;
  double FmMaxA = 2000;
  double FmMinVelocityO = 500;
  double FmCloseL = 100;

  double Bkp = 1E-4; //F is for front
  double Bki = 0; //B is for back
  double Bkd = 0;
  double Bkff = 2.05E-4; //old number 0.7/3500
  double BmMaxV = 6000;
  double BmMaxA = 2000;
  double BmMinVelocityO = 500;
  double BmCloseL = 100;

  const std::string firmwareVersion = "1.8.2"; 
  double FLastkp=0, FLastki=0, FLastkd=0, FLastkff=0;
  double BLastkp=0, BLastki=0, BLastkd=0, BLastkff=0;
  double FarbFF = 0; 
  double BarbFF = 0; 

};