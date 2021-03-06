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
#include <iostream>
#include <frc/SerialPort.h>
#include <sstream>
#include <frc/Timer.h>

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void ConveyorForward();
  void ConveyorForwardRelease();
  void ConveyorBackward();
  void ConveyorBackwardRelease();
  void IntakeIn();
  void IntakeInRelease();
  void IntakeOut();
  void IntakeOutRelease();
  void SensorReset();


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_intakeMotor = rev::CANSparkMax(62, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax m_conveyorMotor = rev::CANSparkMax(6, rev::CANSparkMax::MotorType::kBrushless);
  // rev::CANSparkMax m_loadIntoShooterMotor = rev::CANSparkMax(12, rev::CANSparkMax::MotorType::kBrushless);

  frc::Timer SerialTime;

  bool intakeSigIn = false; 
  bool intakeSigInRelease = false;
  bool intakeSigOutRelease = false;
  bool intakeSigOut = false;
  int stateIntake = 0; 
  double intakeSpeed = 0.0;
  double intakeSpeedOut = 0.4;
  double intakeSpeedIn = 0.7;//original was 0.6

  bool conveyorSigFwd = false;
  bool conveyorSigBack = false;
  bool conveyorSigFwdReleaase = false;
  bool conveyorSigBackRelease = false;
  int stateConveyor = 0;
  //bool sensorDetectsBall = false;
  double stopBallDistance = 0.25;
  bool autoConveyor = false;
  double timeSinceRead = 0.0;
  bool autoConveyorFail = false;

  double conveyorSpeed = 0.0;
  double conveyorSpeedFwd = 0.3;
  double conveyorSpeedBack = 0.3;

  // Serial port for external Arduino (sensors)
  frc::SerialPort m_SerialMXP = frc::SerialPort(115200,frc::SerialPort::kMXP,8,frc::SerialPort::kParity_None,frc::SerialPort::kStopBits_One);

};
