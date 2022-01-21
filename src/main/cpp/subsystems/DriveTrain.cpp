// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain() {

    SetName("DriveTrain");
    myAhrs = new AHRS(frc::SerialPort::kMXP); 
    //RightFront.SetInverted(true);
   // m_robotDrive = new frc::DifferentialDrive(*LeftFront, *RightFront);
    LeftBack.Follow(LeftFront);
    RightBack.Follow(RightFront);

    m_odometry = new frc::DifferentialDriveOdometry{frc::Rotation2d(units::degree_t(GetHeading()))};

    DriveTrain::Reset();

}

void DriveTrain::Drive(double y, double z) {
    m_robotDrive.ArcadeDrive(-y, z);
}

void DriveTrain::autonDrive(){
    m_robotDrive.ArcadeDrive(-.4, 0); 
}

// This method will be called once per scheduler run
void DriveTrain::Periodic() {
    frc::SmartDashboard::PutNumber("Get Heading (ahrs)", myAhrs->GetAngle());
    frc::SmartDashboard::PutNumber("Get Heading (converted)", double(GetHeading()));
  
    frc::SmartDashboard::PutNumber("Output Voltage Right BusVolatage", RightFront.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Output Voltage Left BusVoltage", LeftFront.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Output Voltage Right GetApplied", RightFront.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Output Voltage Left GetApplied", LeftFront.GetAppliedOutput());


    m_odometry->Update(
        frc::Rotation2d(GetHeading()), 
        units::meter_t(LeftEncoder.GetPosition() * 0.044), 
        units::meter_t(-1.0 * RightEncoder.GetPosition() * 0.044)
    );
  
    frc::SmartDashboard::PutNumber("left Encoder Val", LeftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("right Encoder Val", -1.0 * RightEncoder.GetPosition());

  m_field.SetRobotPose(m_odometry->GetPose());
  frc::SmartDashboard::PutData("Field", &m_field);
}

void DriveTrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
//   // if(left > units::volt_t(.25)){ left = units::volt_t(.25); }
//   // if(right > units::volt_t(.25)){ right = units::volt_t(.25); }
  frc::SmartDashboard::PutNumber("Left Distance", LeftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Right Distance", RightEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Tank Drive Volts Left", double(left));
  frc::SmartDashboard::PutNumber("Tank Drive Volts Right", double(right));
  // frc::SmartDashboard::PutNumber("AHRS Heading", GetHeading()); 
  
  LeftFront.SetVoltage(left);
  RightFront.SetVoltage(-right);
  m_robotDrive.Feed();

}
units::degree_t DriveTrain::GetHeading() { 
  return units::degree_t(-1.0 * myAhrs->GetAngle()); // TODO: Fixed Units
}


void DriveTrain::Reset() {
  myAhrs->Reset();
  LeftEncoder.SetPosition(0.0);
  RightEncoder.SetPosition(0.0);
} 

frc::DifferentialDriveWheelSpeeds DriveTrain::GetWheelSpeeds() {
  // units::meter_t(LeftEncoder.GetPosition() * 0.044), 
  // units::meter_t(-1.0 * RightEncoder.GetPosition() * 0.044)
  return {(LeftEncoder.GetVelocity() * 1_mps * 0.044 / 60),
      (-RightEncoder.GetVelocity() * 1_mps * 0.044 / 60)};
}

void DriveTrain::ResetOdometry(frc::Pose2d pose){ 
  Reset(); //reset encoders and ahrs  
  m_odometry->ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading()))); 
}

frc::Pose2d DriveTrain::GetPose(){
  return m_odometry->GetPose();
}

void DriveTrain::ResetEncoders(){ 
  RightEncoder.SetPosition(0); 
  LeftEncoder.SetPosition(0); 
}
