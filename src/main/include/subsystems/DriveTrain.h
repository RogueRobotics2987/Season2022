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
#include <frc2/command/InstantCommand.h>



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
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax LeftBack = rev::CANSparkMax(56, rev::CANSparkMax::MotorType::kBrushless);//56 on Howie, 3 on Jaws
  rev::CANSparkMax LeftFront = rev::CANSparkMax(49, rev::CANSparkMax::MotorType::kBrushless);//49 on Howie, 4 on Jaws
  rev::CANSparkMax RightBack = rev::CANSparkMax(50, rev::CANSparkMax::MotorType::kBrushless);//50 on Howie, 16 on Jaws
  rev::CANSparkMax RightFront = rev::CANSparkMax(46, rev::CANSparkMax::MotorType::kBrushless);//46 on Howie, 1 on Jaws
  frc::DifferentialDrive m_robotDrive{LeftFront, RightFront};

  rev::SparkMaxRelativeEncoder LeftEncoder = LeftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder RightEncoder = RightFront.GetEncoder();

  
  //for autonomous

  // AHRS myAhrs = AHRS(frc::SerialPort::kUSB1);
  frc::DifferentialDriveOdometry m_odometry = frc::DifferentialDriveOdometry{frc::Rotation2d(units::degree_t(GetHeading()))}; 
  frc::Field2d m_field;
};
