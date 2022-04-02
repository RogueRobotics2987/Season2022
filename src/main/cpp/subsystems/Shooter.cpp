// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter(){
    //shooterMotorBack.Follow(shooterMotorFront,false); //false means it is not inverted
    frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", FTargetRPM); 
    frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", BTargetRPM); 
    //F is for front. B is for back
    frc::SmartDashboard::PutNumber("Shooter F Set P", Fkp); 
    frc::SmartDashboard::PutNumber("Shooter F Set I", Fki); 
    frc::SmartDashboard::PutNumber("Shooter F Set D", Fkd); 
    frc::SmartDashboard::PutNumber("Shooter F Set FF", Fkff);
    frc::SmartDashboard::PutNumber("Shooter F Set Arb FF", 0);
    frc::SmartDashboard::PutNumber("Shooter F Set MaxVelcity", FmMaxV);
    frc::SmartDashboard::PutNumber("Shooter F Set MaxAccel", FmMaxA);
    frc::SmartDashboard::PutNumber("Shooter F Set MinVelocityOut", FmMinVelocityO);
    frc::SmartDashboard::PutNumber("Shooter F Set ClosedLoop", FmCloseL);

    frc::SmartDashboard::PutNumber("Shooter B Set P", Bkp); 
    frc::SmartDashboard::PutNumber("Shooter B Set I", Bki); 
    frc::SmartDashboard::PutNumber("Shooter B Set D", Bkd); 
    frc::SmartDashboard::PutNumber("Shooter B Set FF", Bkff);
    frc::SmartDashboard::PutNumber("Shooter B Set Arb FF", 0);
    frc::SmartDashboard::PutNumber("Shooter B Set MaxVelcity", BmMaxV);
    frc::SmartDashboard::PutNumber("Shooter B Set MaxAccel", BmMaxA);
    frc::SmartDashboard::PutNumber("Shooter B Set MinVelocityOut", BmMinVelocityO);
    frc::SmartDashboard::PutNumber("Shooter B Set ClosedLoop", BmCloseL);


    FshooterPID.SetP(Fkp); 
    FshooterPID.SetI(Fki);
    FshooterPID.SetD(Fkd);
    FshooterPID.SetFF(Fkff);
    FshooterPID.SetOutputRange(-1, 1);
    FshooterPID.SetSmartMotionMaxVelocity(FmMaxV);
    FshooterPID.SetSmartMotionMinOutputVelocity(FmMinVelocityO);
    FshooterPID.SetSmartMotionMaxAccel(FmMaxA);
    FshooterPID.SetSmartMotionAllowedClosedLoopError(FmCloseL);
    FshooterPID.SetIZone(800);

    BshooterPID.SetP(Bkp); 
    BshooterPID.SetI(Bki);
    BshooterPID.SetD(Bkd);
    BshooterPID.SetFF(Bkff);
    BshooterPID.SetOutputRange(-1, 1);
    BshooterPID.SetSmartMotionMaxVelocity(BmMaxV);
    BshooterPID.SetSmartMotionMinOutputVelocity(BmMinVelocityO);
    BshooterPID.SetSmartMotionMaxAccel(BmMaxA);
    BshooterPID.SetSmartMotionAllowedClosedLoopError(BmCloseL);
    BshooterPID.SetIZone(800);

}

// This method will be called once per scheduler run
void Shooter::Periodic() {  
    FarbFF = frc::SmartDashboard::GetNumber("Shooter F ArbFF", 0); 
    Fkp = frc::SmartDashboard::GetNumber("Shooter F Set P", Fkp); 
    Fki = frc::SmartDashboard::GetNumber("Shooter F Set I", Fki);
    Fkd = frc::SmartDashboard::GetNumber("Shooter F Set D", Fkd);
    Fkff = frc::SmartDashboard::GetNumber("Shooter F Set FF", Fkff);

    BarbFF = frc::SmartDashboard::GetNumber("Shooter B ArbFF", 0); 
    Bkp = frc::SmartDashboard::GetNumber("Shooter B Set P", Bkp); 
    Bki = frc::SmartDashboard::GetNumber("Shooter B Set I", Bki);
    Bkd = frc::SmartDashboard::GetNumber("Shooter B Set D", Bkd);
    Bkff = frc::SmartDashboard::GetNumber("Shooter B Set FF", Bkff);

    //NOTE: This doesn't update speed until you click the "shooter on" button
    FTargetRPM = frc::SmartDashboard::GetNumber("Shooter Set RPM 2 F", FTargetRPM); 
    BTargetRPM = frc::SmartDashboard::GetNumber("Shooter Set RPM 2 B", BTargetRPM); 

    frc::SmartDashboard::PutNumber("Shooter Front speed", shooterEncoderFront.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter Back speed", shooterEncoderBack.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter App Out", shooterMotorFront.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Shooter Applied Current", shooterMotorFront.GetOutputCurrent());
    if(FLastkp != Fkp)   {FshooterPID.SetP(Fkp);   FLastkp = Fkp;}
    if(FLastki != Fki)   {FshooterPID.SetI(Fki);   FLastki = Fki;}
    if(FLastkd != Fkd)   {FshooterPID.SetD(Fkd);   FLastkd = Fkd;}
    if(FLastkff != Fkff) {FshooterPID.SetFF(Fkff); FLastkff = Fkff;}

    if(BLastkp != Bkp)   {BshooterPID.SetP(Bkp);   BLastkp = Bkp;}
    if(BLastki != Bki)   {BshooterPID.SetI(Bki);   BLastki = Bki;}
    if(BLastkd != Bkd)   {BshooterPID.SetD(Bkd);   BLastkd = Bkd;}
    if(BLastkff != Bkff) {BshooterPID.SetFF(Bkff); BLastkff = Bkff;}

}

double Shooter::getVelocity(){
    return shooterEncoderFront.GetVelocity(); 
    //return shooterEncoderBack.GetVelocity();
}

void Shooter::setPercent(double percent){
    shooterMotorFront.Set(percent); 
    shooterMotorBack.Set(percent); 
}

void Shooter::startShooter() {
    shooterMotorFront.Set(0.2);
    shooterMotorBack.Set(0.2);
}

void Shooter::stopShooter() {
    shooterMotorFront.Set(0);
    shooterMotorBack.Set(0);
}

void Shooter::setShooter() {
   // shooterPID->SetReference(maxRPM, rev::ControlType::kVelocity);
   // shooterPID.SetReference(TargetRPM, rev::ControlType::kVelocity, arbFF);

//NOTE: This update speeds only when you actually click the "shooter on" button
    FshooterPID.SetReference(-FTargetRPM, rev::ControlType::kSmartVelocity, FarbFF);
    BshooterPID.SetReference(-BTargetRPM, rev::ControlType::kSmartVelocity, BarbFF);

}