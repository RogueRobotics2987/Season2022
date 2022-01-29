// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(){
frc::SmartDashboard::PutNumber("Set P", kp); 
    frc::SmartDashboard::PutNumber("Set I", ki); 
    frc::SmartDashboard::PutNumber("Set D", kd); 
    frc::SmartDashboard::PutNumber("Set FF", kff);
    frc::SmartDashboard::PutNumber("Set Arb FF", 0);

    shooterPID.SetP(kp); 
    shooterPID.SetI(ki);
    shooterPID.SetD(kd);
    //shooterPID->SetP(.0016/2); //Ollie motor only
    shooterPID.SetOutputRange(-1, 1);
    shooterPID.SetSmartMotionMaxVelocity(4000);
    shooterPID.SetSmartMotionMinOutputVelocity(1500);
    shooterPID.SetSmartMotionMaxAccel(1000.0/1.0);
    shooterPID.SetSmartMotionAllowedClosedLoopError(0.0);
    shooterPID.SetIZone(800);
    shooterPID.SetFF(0.7/3500);

    myTimer = frc::Timer();
    myTimer.Reset();
    myTimer.Start();
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
    bool shooterWorks = true; 

    units::time::second_t startTime = myTimer.Get();

    if(shooterMotor.GetFirmwareString() != firmwareVersion){
        shooterWorks = false;
    }
    frc::SmartDashboard::PutBoolean("Shooter Works", shooterWorks); 
    arbFF = frc::SmartDashboard::GetNumber("ArbFF", 0); 
    kp = frc::SmartDashboard::GetNumber("Set P", kp); 
    ki = frc::SmartDashboard::GetNumber("Set I", ki);
    kd = frc::SmartDashboard::GetNumber("Set D", kd);
    kff = frc::SmartDashboard::GetNumber("Set FF", kff);

    TargetRPM = frc::SmartDashboard::GetNumber("Set RPM", 3800); 
    frc::SmartDashboard::PutNumber("Shooter speed", shooterEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter App Out", shooterMotor.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Shooter Applied Current", shooterMotor.GetOutputCurrent());
    if(Lastkp != kp)   {shooterPID.GetSmartMotionAccelStrategy(kp);   Lastkp = kp;}
    if(Lastki != ki)   {shooterPID.SetI(ki);   Lastki = ki;}
    if(Lastkd != kd)   {shooterPID.SetD(kd);   Lastkd = kd;}
    if(Lastkff != kff) {shooterPID.SetFF(kff); Lastkff = kff;}

    units::time::second_t EndTime = myTimer.Get();
    units::time::second_t RunTime= EndTime - startTime;
    if(0) {
        std::cout << "Shooter Periodic Time: " << RunTime.value() << std::endl;
    }
    

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

void Shooter::setShooter(double RPM) {
   // shooterPID->SetReference(maxRPM, rev::ControlType::kVelocity);
    shooterPID.SetReference(TargetRPM, rev::ControlType::kVelocity, arbFF);
}