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
    frc::SmartDashboard::PutNumber("ClimbRightPosition", re_climbMotorRight.GetPosition());
    frc::SmartDashboard::PutNumber("ClimbLeftPosition", re_climbMotorLeft.GetPosition());
   // frc::SmartDashboard::PutBoolean("Climb Left Limit",ls_climbLeft.Get());
    //frc::SmartDashboard::PutBoolean("Climb Right Limit", ls_climbRight.Get());

    //changes from ClimberTweaks branch
    double base_error = re_climbMotorRight.GetPosition() - re_climbMotorLeft.GetPosition();
    double error_kp = climbKValue*base_error;
    //m_climbMotorRight.Set(climbVal);
    if (error_kp > 0.2) {
        error_kp = 0.2;
    }
    if (error_kp < -0.2) {
        error_kp = -0.2;
    }



    frc::SmartDashboard::PutNumber("ClimbErrorBase", base_error);
    frc::SmartDashboard::PutNumber("ClimbErrorToMotor", error_kp);

    /*if (ls_climbRight.Get() != true) {
        m_climbMotorRight.Set(climbVal - error_kp);
    } else {
        m_climbMotorRight.Set(0);
    }
    if (ls_climbLeft.Get() != true) {
        m_climbMotorLeft.Set(climbVal + error_kp); //comment out

    }*/
    m_climbMotorRight.Set(climbVal - error_kp); // comment out
    m_climbMotorLeft.Set(climbVal + error_kp); //comment out

    //m_climbMotorRight.Set(climbVal);
    //m_climbMotorLeft.Set(climbVal);
    
    frc::SmartDashboard::PutNumber("ClimbRight", RC); 
    frc::SmartDashboard::PutNumber("ClimbLeft", LC); 
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
void Climber::ClimbServoLock(){
    m_climbServoRight.SetAngle(90); //don't know on angles
    m_climbServoLeft.SetAngle(90);
}
void Climber::ClimbServoUnlock(){
    m_climbServoRight.SetAngle(0); //don't know on angles
    m_climbServoLeft.SetAngle(0);
}