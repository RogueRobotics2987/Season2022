// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber(){
    //m_climbMotorLeft.Follow(m_climbMotorRight, true); //comment out
    frc::SmartDashboard::PutNumber("climbKValue", climbKValue);
}

// This method will be called once per scheduler run
void Climber::Periodic() {
    // climbVal = 0.0;
    frc::SmartDashboard::PutNumber("climbVal", climbVal);
    climbKValue = frc::SmartDashboard::GetNumber("climbKValue", climbKValue);

}
//int myFunc(double in) { return in*3;}
    
void Climber::ClimbFunction(double climbUpVal, double climbDownVal){
    frc::SmartDashboard::PutNumber("climbUpVal", climbUpVal);
    frc::SmartDashboard::PutNumber("climbDownVal", climbDownVal);

    if((fabs(climbUpVal) > upDeadzone) && (climbDownVal < downDeadzone)){
        climbVal = climbUpVal - upDeadzone;
    } else if((fabs(climbDownVal) > downDeadzone) && (climbUpVal < upDeadzone)){
        climbVal = -(climbDownVal - downDeadzone);
    } else {
        climbVal = 0;
    }

    // climbVal = 0.0
}
void Climber::ClimbServoLock(){
    m_climbServoRight.SetAngle(270); //don't know on angles
    // m_climbServoLeft.SetAngle(90);
}
void Climber::ClimbServoUnlock(){
    m_climbServoRight.SetAngle(0); //don't know on angles
    // m_climbServoLeft.SetAngle(0);
}