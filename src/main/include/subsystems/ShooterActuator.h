// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>



class ShooterActuator : public frc2::SubsystemBase {
 public:
  ShooterActuator();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  void setAngleH(double stickVal);
  void setAngleV(double stickVal);
  bool GetForwardLimitState();
  void SetAutoAim(bool AutoAimFlag);
  void safeSetH(double setVal);
  void safeSetV(double setVal);
  double GetTY(); 
  double GetTX(); 
  void switchCam(bool flag); 
  void limeStream(int num);

 private:
  rev::CANSparkMax angleMotorH=  rev::CANSparkMax(54, rev::CANSparkMax::MotorType::kBrushless);
  rev::CANSparkMax angleMotorV= rev::CANSparkMax(59, rev::CANSparkMax::MotorType::kBrushless);  
  
  //std::shared_ptr<nt::NetworkTable> limelightTable;
  int H_AimState = 0;
  int V_AimState = 0;
  float tx = 0;
  float ty = 0;
  float AimH_P = 0.32;
  float AimV_P = 0.32;
  float PositionH;
  float PositionV;
  frc::Timer myTimer;

  bool AutoAimMode = false;
  double safeStick(double stickVal, double pos);
  std::string firmwareVersion = "1.8.2"; 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
