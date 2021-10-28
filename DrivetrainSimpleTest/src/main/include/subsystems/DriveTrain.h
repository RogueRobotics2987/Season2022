// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include "AHRS.h"
#include <frc/smartdashboard/Field2d.h>
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>




class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  void Log();
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();
  /**
   * Tank style driving for the DriveTrain.
   * @param left Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  void Drive(double y, double z);

  /**
   * @return The robots heading in degrees.
   */
  double GetHeading();
  void ResetOdometry(frc::Pose2d pose); 

  /**
   * Reset the robots sensors to the zero states.
   */
  void Reset();

  /**
   * @return The distance driven (average of left and right encoders).
   */
  double GetDistance();

  /**
   * @return The distance to the obstacle detected by the rangefinder.
   */
  double GetDistanceToObstacle();
  double GetTurnRate(); 

frc::Pose2d GetPose(); 
void ResetEncoders(); 
void TrajectoryInit(); 
frc::DifferentialDriveWheelSpeeds GetWheelSpeeds(); 
void TankDriveVolts(units::volt_t left, units::volt_t right); 


 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
    rev::CANSparkMax* LeftBack = new rev::CANSparkMax(56, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax* LeftFront = new rev::CANSparkMax(49, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANEncoder LeftEncoder = LeftFront->GetEncoder(); 
    rev::CANSparkMax* RightBack = new rev::CANSparkMax(50, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANSparkMax* RightFront = new rev::CANSparkMax(46, rev::CANSparkMax::MotorType::kBrushless);
    rev::CANEncoder RightEncoder = RightFront->GetEncoder();

    frc::DifferentialDrive* m_robotDrive = nullptr;
    AHRS* myAhrs = nullptr; 
    frc::DifferentialDriveOdometry* m_odometry = nullptr; 

    frc::Field2d m_field;

};
