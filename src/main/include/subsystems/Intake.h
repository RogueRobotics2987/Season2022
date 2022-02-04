// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h> 
#include <frc/Solenoid.h> 
#include <frc/DoubleSolenoid.h> 
#include <frc/Timer.h> 
#include <iostream>
class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void StartConveyor(double percent); 
  void IntakeBall(double setVal);
  void PrepareBall();
  void StopMotors();
  void setSolenoidTrue(); 
  void setSolenoidFalse(); 
  void startTimer(); 
  void ResetBallCount(); 
  double conveyorVal; 
  void resetOutBalls(); 

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax p_intakeMotor = rev::CANSparkMax(51, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax p_conveyorMotor = rev::CANSparkMax(48, rev::CANSparkMax::MotorType::kBrushless);
  frc::DigitalInput p_intakeSensor = frc::DigitalInput(1);
  frc::DigitalInput p_topSensor = frc::DigitalInput(2); 
  frc::Timer myTimer; 
  frc::Timer myTimer2;
  bool timeGotten = false; 
  units::time::second_t conveyorTime = 0_s; 
  int ballCount = 0;
  bool sensorBool = false; 
  bool secondSensorBool = false; 
  bool firstTimeGotten = false; 
  int ballOut = 0; 
  bool intakeTime = 0; 
  units::time::second_t firstTime; 
  frc::DoubleSolenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 0, 1};  
  std::string firmwareVersion = "1.8.2"; 
  frc::Timer intakeTimer;
  bool intakeGood = true; 
  bool sensorSafety = false; 
};