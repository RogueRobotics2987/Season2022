// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "AHRS.h"
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h" 
#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/angle.h>
#include "Constants.h" 
#include "frc/controller/PIDController.h"


class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  void Drive(double y, double z);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void autonDrive(); 

  /**
  * @return The robots heading in degrees.
  */
  units::degree_t GetHeading();
  void ResetOdometry(frc::Pose2d pose);

  /**
  * Reset the robots sensors to the zero states.
  */
  void Reset();

  frc::Pose2d GetPose(); 
  void ResetEncoders(); 
  void TrajectoryInit(); 
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds(); 
  void TankDriveVolts(units::volt_t left, units::volt_t right);

 private:
  double Lvkp = 0.1;
  double Lvki = 0.001;
  double Lvkd = 0;

  double Avkp = 0.1;
  double Avki = 0.001;
  double Avkd = 0;
  frc2::PIDController LvPid{Lvkp, Lvki, Lvkd};
  frc2::PIDController AvPid{Avkp, Avki, Avkd};

  double currentAngle = 0;
  double currentDistance = 0;


  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax LeftBack = rev::CANSparkMax(3, rev::CANSparkMax::MotorType::kBrushless);//56 on Howie
  rev::CANSparkMax LeftFront = rev::CANSparkMax(4, rev::CANSparkMax::MotorType::kBrushless);//49 on Howie
  rev::CANSparkMax RightBack = rev::CANSparkMax(2, rev::CANSparkMax::MotorType::kBrushless);//50 on Howie
  rev::CANSparkMax RightFront = rev::CANSparkMax(1, rev::CANSparkMax::MotorType::kBrushless);//46 on Howie
  frc::DifferentialDrive m_robotDrive{LeftFront, RightFront};

  rev::SparkMaxRelativeEncoder LeftEncoder = LeftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder RightEncoder = RightFront.GetEncoder();

  
  //for autonomous
  AHRS myAhrs = AHRS( frc::SerialPort::kMXP); 
  frc::DifferentialDriveOdometry m_odometry = frc::DifferentialDriveOdometry{frc::Rotation2d(units::degree_t(GetHeading()))}; 
  frc::Field2d m_field;
};
