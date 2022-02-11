// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"

DriveTrain::DriveTrain() {

    SetName("DriveTrain");
    LeftBack.Follow(LeftFront);
    RightBack.Follow(RightFront);
    LeftFront.SetInverted(true);
    RightFront.SetInverted(false); 
    DriveTrain::Reset();


    frc::SmartDashboard::PutNumber("Set P", Lvkp); 
    frc::SmartDashboard::PutNumber("Set I", Lvki); 
    frc::SmartDashboard::PutNumber("Set D", Lvkd);
    
    frc::SmartDashboard::PutNumber(" P", Avkp); 
    frc::SmartDashboard::PutNumber(" I", Avki); 
    frc::SmartDashboard::PutNumber(" D", Avkd);

    frc::SmartDashboard::PutNumber("CurrentDistance",currentPitch );
    frc::SmartDashboard::PutNumber("CurrentAngle",currentHeading );

  //  DrivePID = new rev::CANPIDController(shooterMotor);

}

void DriveTrain::Drive(double xSpeed, double zRotation) {
    m_robotDrive.ArcadeDrive(xSpeed, -zRotation);
}

void DriveTrain::autonDrive(){
    m_robotDrive.ArcadeDrive(-.4, 0); 
}

// This method will be called once per scheduler run
void DriveTrain::Periodic() {

    std::vector< double > xArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumberArray("Left X", defaultValReturn);
    std::vector< double > yArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumberArray("Top Y",defaultValReturn);
    std::vector< std::string > labelArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetStringArray("Label",defaultStringReturn);
    std::vector< double > areaArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumberArray("Area",defaultValReturn);
    std::vector< double > confArray = nt::NetworkTableInstance::GetDefault().GetTable("Jetson")->GetNumberArray("Confidence",defaultValReturn);

  //  currentHeading = frc::SmartDashboard::GetNumber("CurrentHeading", currentHeading);
  //  currentPitch = frc::SmartDashboard::GetNumber("CurrentPitch", currentPitch);
    currentHeading = CalculateTheta(xArray[0]);
    currentPitch = CalculatePhi(yArray[0]);

    frc::SmartDashboard::PutNumber("UpdatedCurrentAngle", currentHeading);
    frc::SmartDashboard::PutNumber("UpdatedCurrentDistance", currentPitch);

    frc::SmartDashboard::PutNumber("ballx", xArray[0]);
    frc::SmartDashboard::PutNumber("bally", yArray[0]);
    frc::SmartDashboard::PutNumberArray("ballxArray", xArray);


    double LvPidOut = LvPid.Calculate(currentPitch ,0);
    double AvPidOut = AvPid.Calculate(currentHeading ,0);

    if (AvPidOut > 0.2){
      AvPidOut = 0.2;
    } else if (AvPidOut < -0.2){
      AvPidOut = -0.2;
    }

    m_robotDrive.ArcadeDrive(0.2, AvPidOut); 

    frc::SmartDashboard::PutNumber("LvPid", LvPidOut);
    frc::SmartDashboard::PutNumber("AvPid", AvPidOut);



    frc::SmartDashboard::PutNumber("Get Heading (ahrs)", myAhrs.GetAngle());
    frc::SmartDashboard::PutNumber("Get Heading (converted)", double(GetHeading()));
  
    frc::SmartDashboard::PutNumber("Output Voltage Right BusVolatage", RightFront.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Output Voltage Left BusVoltage", LeftFront.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Output Voltage Right GetApplied", RightFront.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Output Voltage Left GetApplied", LeftFront.GetAppliedOutput());


    m_odometry.Update(
        frc::Rotation2d(GetHeading()), 
        units::meter_t(LeftEncoder.GetPosition() * 0.044), 
        units::meter_t(-1.0 * RightEncoder.GetPosition() * 0.044)
    );
  
    frc::SmartDashboard::PutNumber("left Encoder Val", LeftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("right Encoder Val", -1.0 * RightEncoder.GetPosition());

  m_field.SetRobotPose(m_odometry.GetPose());
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
  return units::degree_t(-1.0 * myAhrs.GetAngle()); // TODO: Fixed Units
}


void DriveTrain::Reset() {
  myAhrs.Reset();
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
  m_odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading()))); 
}

frc::Pose2d DriveTrain::GetPose(){
  return m_odometry.GetPose();
}

void DriveTrain::ResetEncoders(){ 
  RightEncoder.SetPosition(0); 
  LeftEncoder.SetPosition(0); 
}

double DriveTrain::CalculatePhi(double current_y){
  double phi;
  phi = fov_y*current_y/max_y;
  return phi;
}

double DriveTrain::CalculateTheta(double current_x){
  double theta;
  theta = current_x*fov_x/max_x;
  return theta;
}