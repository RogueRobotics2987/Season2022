// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"

Climber::Climber(){

}

// This method will be called once per scheduler run
void Climber::Periodic() {
}

void Climber::ClimbUp(double climbUpVal){
 
    if(fabs(climbUpVal) < .08){
        climbUpVal = 0;
    }
 
    m_climbMotor1.Set(climbUpVal);
}
 
void Climber::ClimbDown(double climbDownVal){
    if(fabs(climbDownVal) < .08){
        climbDownVal = 0;
    }
 
    m_climbMotor1.Set(-climbDownVal);
}
 
