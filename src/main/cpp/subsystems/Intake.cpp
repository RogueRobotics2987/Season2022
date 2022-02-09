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
        //Intake in
        m_intakeMotor.Set(0.5); //50% speed 
        intakeSigIn = false;
        stateConveyor = 1;
        //exit stateIntakes
        if(intakeSigInRelease) {
            stateIntake = 3;
        } else if (intakeSigOut) {
            stateIntake = 2;
        }
    } else if (stateIntake == 2) {
        //Intake out
        m_intakeMotor.Set(-0.5);
        intakeSigOut = false;
        
        //exit statements
        if (intakeSigOutRelease){
            stateIntake = 3;
        } else if (intakeSigIn) {
            stateIntake = 1;
        }
    } else if (stateIntake == 3){
        //stopped intake
        m_intakeMotor.Set(0.0);

        //exit statements 
        if (intakeSigIn) {
            stateIntake = 1;
        } else if (intakeSigOut){
            stateIntake = 3;
        }
    } 

    //conveyor state machine
    if (stateConveyor == 0){
        //initialization state
        m_conveyorMotor.Set(0.0);
        stateConveyor = 3;
    } else if (stateConveyor == 1){
        //conveyor forward
        m_conveyorMotor.Set(0.5); //50% speed
        conveyorSigFwd = false;

        if (conveyorSigFwdReleaase){
            stateConveyor = 3;
        } else if (conveyorSigBack){
            stateConveyor = 2;
        }
    } else if (stateConveyor = 2){
        //conveyor backward
        m_conveyorMotor.Set(-0.5);
        conveyorSigBack = false;

        if (conveyorSigBackRelease){
            stateConveyor = 3;
        } else if (conveyorSigFwd){
            stateConveyor = 1;
        }
    } else if (stateConveyor = 3){
        //conveyor moter stopped
        m_conveyorMotor.Set(0.0);

        if (conveyorSigFwd){
            stateConveyor = 1;
        } else if (conveyorSigBack){
            stateConveyor = 2;
        }
    }



}

void Intake::IntakeIn(){
   // Set:ConveyorForward
   intakeSigIn = true;
}

void Intake::IntakeOut(){
    intakeSigOut = true;
    //m_intakeMotor.Set(0.5);
}

void Intake::ConveyorForward(){
    conveyorSigFwd = true;
}

void Intake::ConveyorBackward(){
    conveyorSigBack = true;
}

void Intake::IntakeBall(double setVal){
    m_intakeMotor.Set(setVal);
}

void Intake::StartConveyor(double percent) {
    m_conveyorMotor.Set(percent);
}