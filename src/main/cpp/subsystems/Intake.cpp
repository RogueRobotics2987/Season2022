// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() {

}

void Intake::setSolenoidTrue(){
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}
void Intake::setSolenoidFalse(){
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}
// This method will be called once per scheduler run
void Intake::Periodic() {

}

void Intake::IntakeBall(double setVal){
    m_intakeMotor.Set(setVal);
}

void Intake::StartConveyor(double percent) {
    m_conveyorMotor.Set(percent);
}