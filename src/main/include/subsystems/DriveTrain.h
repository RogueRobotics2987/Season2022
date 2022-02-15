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
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

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

  double CalculatePhi(double current_y);
  double CalculateTheta(double current_x);

  void JetsonControl();
 
  double calculateBallAverage(std::vector< double > * ball_Array);
 private:

  double Lvkp = 0.012;
  double Lvki = 0.0001;
  double Lvkd = 0;

  double Avkp = 0.008;
  double Avki = 0.0001;
  double Avkd = 0;
  frc2::PIDController LvPid{Lvkp, Lvki, Lvkd};
  frc2::PIDController AvPid{Avkp, Avki, Avkd};

  double AvPidOut = 0;
  double LvPidOut = 0;

  double currentHeading = 33;
  double currentPitch = 8;

  double fov_x = 90;
  double fov_y = 10*fov_x/19;

  double max_x = 640;
  double max_y = 480;

  //jetson stuff
  const double defaultValReturn[2] = {0.0, 0.0};
  const std::string defaultStringReturn[2] = {"not", "found"};

  double X_THRESH = 30;
  double Y_THRESH = 30;


  std::vector< double > firstball_x = {};
  std::vector< double > secondball_x = {};
  std::vector< double > thirdball_x ={};

  std::vector< double > firstball_y = {};
  std::vector< double > secondball_y = {};
  std::vector< double > thirdball_y ={};

  int num_x_0 = 0;
  double last_x_val = 0;
  double max_undetected_iterations = 5;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax LeftBack = rev::CANSparkMax(56, rev::CANSparkMax::MotorType::kBrushless);//56 on Howie
  rev::CANSparkMax LeftFront = rev::CANSparkMax(49, rev::CANSparkMax::MotorType::kBrushless);//49 on Howie
  rev::CANSparkMax RightBack = rev::CANSparkMax(50, rev::CANSparkMax::MotorType::kBrushless);//50 on Howie
  rev::CANSparkMax RightFront = rev::CANSparkMax(46, rev::CANSparkMax::MotorType::kBrushless);//46 on Howie
  frc::DifferentialDrive m_robotDrive{LeftFront, RightFront};

  rev::SparkMaxRelativeEncoder LeftEncoder = LeftFront.GetEncoder();
  rev::SparkMaxRelativeEncoder RightEncoder = RightFront.GetEncoder();

  
  //for autonomous
  AHRS myAhrs = AHRS( frc::SerialPort::kMXP); 
  frc::DifferentialDriveOdometry m_odometry = frc::DifferentialDriveOdometry{frc::Rotation2d(units::degree_t(GetHeading()))}; 
  frc::Field2d m_field;
};
