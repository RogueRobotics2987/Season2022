// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber(){
    m_climbMotorLeft.Follow(m_climbMotorRight, true);
}

// This method will be called once per scheduler run
void Climber::Periodic() {
    // climbVal = 0.0;
    frc::SmartDashboard::PutNumber("climbVal", climbVal);

    m_climbMotorRight.Set(climbVal);
    frc::SmartDashboard::PutNumber("RightClimb", RC); 
    frc::SmartDashboard::PutNumber("LeftClimb", LC); 
    m_climbMotorLeft.GetOutputCurrent();
    m_climbMotorRight.GetOutputCurrent();
    RC = m_climbMotorRight.GetOutputCurrent();
    LC = m_climbMotorLeft.GetOutputCurrent();


   /* double bob = 4.33;  // Amps
    double bob = m_climbMotorLeft.GetOutputCurrent();  // Amps

    frc::SmartDashboard::PutNumber("LeftClimb", bob); */
    
    
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

    // climbVal = 0.0;

}
