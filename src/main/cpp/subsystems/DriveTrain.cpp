// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"
#include <string>
#include <iostream>

float rrsDecoder(std::string inputArray) {
        static float output = 0.0;
        std::string cmString = "";
        int cmInt = 0;
        // Finds the position of "B" marking the beginning of the range value
        int BIndex = inputArray.find("B");
        BIndex += 1; // Prevents "B" from being added to the return value
        // Adds on to cmString from inputArray until an "E" is reached
        
        // Converts cmInt string into int, converted to mm as a float
       if (inputArray == "" || inputArray == "B" || inputArray == "E" || inputArray == "BB" || inputArray == "BE" || inputArray == ","){
         cmString = "0";
       } else {
        while (inputArray[BIndex] != 'E') {
            cmString += inputArray[BIndex];
            BIndex++;
          }
        }
        if(!(cmString == "0")) {
          cmInt = stoi(cmString);
          output = cmInt/100.0;
          std::cout << output << std::endl;
        }
        frc::SmartDashboard::PutNumber("Range", output);
        return output;
}

DriveTrain::DriveTrain() {

    SetName("DriveTrain");
    LeftBack.Follow(LeftFront);
    RightBack.Follow(RightFront);
    LeftFront.SetInverted(true);
    RightFront.SetInverted(false); 
    DriveTrain::Reset();
    DriveTrain::rraReset();

}

void DriveTrain::Drive(double xSpeed, double zRotation) {
    m_robotDrive.ArcadeDrive(xSpeed, -zRotation);
}

void DriveTrain::rraReset() {
    m_SerialMXP.SetTimeout(units::time::second_t(0.001));
    m_SerialMXP.SetReadBufferSize(5);
    m_SerialMXP.Reset();
}

void DriveTrain::autonDrive(){
    m_robotDrive.ArcadeDrive(-.4, 0); 
}

// This method will be called once per scheduler run
void DriveTrain::Periodic() {
    //frc::SmartDashboard::PutNumber("Get Heading (ahrs)", myAhrs.GetAngle());
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
    
    char sSenseData[10] = {NULL};
    int bytesRead = 0;
    bytesRead = m_SerialMXP.Read(sSenseData,9);
    sSenseData[9] = NULL;
    std::string soSenseData = sSenseData;
    float fSenseData = rrsDecoder(soSenseData);
    frc::SmartDashboard::PutString("myKey",sSenseData);
    frc::SmartDashboard::PutNumber("bytesRead",bytesRead);
    //frc::SmartDashboard::PutNumber("Range", fSenseData);
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
  return units::degree_t(0.0); // Temp Removed -1.0 * myAhrs.GetAngle()); // TODO: Fixed Units
}


void DriveTrain::Reset() {
  //myAhrs.Reset();
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
