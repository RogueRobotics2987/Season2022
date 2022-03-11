// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h" 
#include <frc/Joystick.h>
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class TurretSubsystem : public frc2::SubsystemBase {
 public:
  TurretSubsystem();
  void setSpeed(float speed);
  void setAngleV(float l_stickValV);
  void setAngleH(float l_stickValH);
  void setAutoAimOn();
  
  void setManuelAimOn();
  void setStickPOV(int stickPOV);


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  private:

    frc::Joystick* m_xBox = nullptr;

    // rev::CANSparkMax m_turret = rev::CANSparkMax(60, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax m_vTurretMotorRight{13, rev::CANSparkMax::MotorType::kBrushless}; //original motor
    rev::CANSparkMax m_vTurretMotorLeft{44, rev::CANSparkMax::MotorType::kBrushless}; //follower motor

    rev::CANSparkMax m_hTurretMotor{14, rev::CANSparkMax::MotorType::kBrushless}; 
    rev::SparkMaxLimitSwitch ls_vTurretMotorRight = m_vTurretMotorRight.GetForwardLimitSwitch(
                              rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
    rev::SparkMaxLimitSwitch ls_vTurretMotorLeft = m_vTurretMotorLeft.GetForwardLimitSwitch(
                              rev::SparkMaxLimitSwitch::Type::kNormallyClosed);
    // rev::SparkMaxLimitSwitch ls_hTurretMotor = m_hTurretMotor.GetForwardLimitSwitch(
    //                              rev::SparkMaxLimitSwitch::LimitSwitchPolarity::kNormallyClosed);

    rev::SparkMaxRelativeEncoder re_vTurretMotorRight = m_vTurretMotorRight.GetEncoder();
    rev::SparkMaxRelativeEncoder re_vTurretMotorLeft = m_vTurretMotorLeft.GetEncoder();

    enum TurretState_t {INIT, R_BOTH, R_LEFT, R_RIGHT, DRIVER_SHOOT, AUTO_SHOOT, PRESHOOT_RAISE}; //R_ means retract
    TurretState_t TurretState = R_BOTH;

    double cur_stickValV = 0.0;
    double cur_stickValH = 0.0;
    double kp_hAim = 0.01;
    double kp_vAimty = 1.0;
    double kp_vAimre = 0.02;
    double turretScaleVal = 0.4; //percent of max speed 

    int cur_pipeline = 3;
    int cur_stickPOV = 0;
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.
};
