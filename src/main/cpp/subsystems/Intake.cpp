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
    if (stateIntake == 0) {
        //initialization
        //reset timers
        m_intakeMotor.Set(0.0); //also put in stateIntake 3

        //clean up ball counts or bools
        stateIntake = 3; //stopped stateIntake
    } else if (stateIntake == 1) {
        //goForward
        m_intakeMotor.Set(0.5);
        intakeSigFwd = false;
        stateConveyor = 1;
        //exit stateIntakes
        if(intakeSigRelease) {
            stateIntake = 2;
        } else if (intakeSigBack) {
            stateIntake = 3;
        }
    }
}

void Intake::ConveyorForward(){
   // Set:ConveyorForward
   intakeSigFwd = true;
}

void Intake::ConveyorBackward(){
    m_intakeMotor.Set(0.5);
}

void Intake::IntakeBall(double setVal){
    m_intakeMotor.Set(setVal);
}

void Intake::StartConveyor(double percent) {
    m_conveyorMotor.Set(percent);
}