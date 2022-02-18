// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(){
    int bob = 1;
    frc::SmartDashboard::PutNumber("Set RPM 2", TargetRPM); 
    frc::SmartDashboard::PutNumber("Set RPM", TargetRPM); 
    frc::SmartDashboard::PutNumber("Set P", kp); 
    frc::SmartDashboard::PutNumber("Set I", ki); 
    frc::SmartDashboard::PutNumber("Set D", kd); 
    frc::SmartDashboard::PutNumber("Set FF", kff);
    frc::SmartDashboard::PutNumber("Set Arb FF", 0);
    frc::SmartDashboard::PutNumber("Set MaxVelcity", mMaxV);
    frc::SmartDashboard::PutNumber("Set MaxAccel", mMaxA);
    frc::SmartDashboard::PutNumber("Set MinVelocityOut", mMinVelocityO);
    frc::SmartDashboard::PutNumber("Set ClosedLoop", mCloseL);

    shooterPID.SetP(kp); 
    shooterPID.SetI(ki);
    shooterPID.SetD(kd);
    shooterPID.SetFF(kff);
    shooterPID.SetOutputRange(-1, 1);
    shooterPID.SetSmartMotionMaxVelocity(mMaxV);
    shooterPID.SetSmartMotionMinOutputVelocity(mMinVelocityO);
    shooterPID.SetSmartMotionMaxAccel(mMaxA);
    shooterPID.SetSmartMotionAllowedClosedLoopError(mCloseL);
    shooterPID.SetIZone(800);

   
}

// This method will be called once per scheduler run
void Shooter::Periodic() {  
    arbFF = frc::SmartDashboard::GetNumber("ArbFF", 0); 
    kp = frc::SmartDashboard::GetNumber("Set P", kp); 
    ki = frc::SmartDashboard::GetNumber("Set I", ki);
    kd = frc::SmartDashboard::GetNumber("Set D", kd);
    kff = frc::SmartDashboard::GetNumber("Set FF", kff);

    TargetRPM = frc::SmartDashboard::GetNumber("Set RPM 2", TargetRPM); 
    frc::SmartDashboard::PutNumber("Shooter speed", shooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter App Out", shooterMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Shooter Applied Current", shooterMotor.GetOutputCurrent());
    if(Lastkp != kp)   {shooterPID.GetSmartMotionAccelStrategy(kp);   Lastkp = kp;}
    if(Lastki != ki)   {shooterPID.SetI(ki);   Lastki = ki;}
    if(Lastkd != kd)   {shooterPID.SetD(kd);   Lastkd = kd;}
    if(Lastkff != kff) {shooterPID.SetFF(kff); Lastkff = kff;}
}

double Shooter::getVelocity(){
    return shooterEncoder.GetVelocity(); 
}

void Shooter::setPercent(double percent){
    shooterMotor.Set(percent); 
}

void Shooter::startShooter() {
    shooterMotor.Set(0.2);
}

void Shooter::stopShooter() {
    shooterMotor.Set(0);
}

void Shooter::setShooter() {
   // shooterPID->SetReference(maxRPM, rev::ControlType::kVelocity);
   // shooterPID.SetReference(TargetRPM, rev::ControlType::kVelocity, arbFF);
    shooterPID.SetReference(-TargetRPM, rev::ControlType::kSmartVelocity, arbFF);
}