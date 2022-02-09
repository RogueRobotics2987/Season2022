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

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void setSolenoidTrue();
  void setSolenoidFalse();
  //void StartConveyor(double percent);
  //void IntakeBall(double setVal);
  void ConveyorForward();
  void ConveyorForwardRelease();
  void ConveyorBackward();
  void ConveyorBackwardRelease();
  void IntakeIn();
  void IntakeInRelease();
  void IntakeOut();
  void IntakeOutRelease();


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_intakeMotor = rev::CANSparkMax(51, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax m_conveyorMotor = rev::CANSparkMax(48, rev::CANSparkMax::MotorType::kBrushless);
  frc::DoubleSolenoid intakeSolenoid {frc::PneumaticsModuleType::CTREPCM, 0, 1};
  bool intakeSigIn = false; 
  bool intakeSigInRelease = false;
  bool intakeSigOutRelease = false;
  bool intakeSigOut = false;
  int stateIntake = 0; 

  bool conveyorSigFwd = false;
  bool conveyorSigBack = false;
  bool conveyorSigFwdReleaase = false;
  bool conveyorSigBackRelease = false;
  int stateConveyor = 0;
};
