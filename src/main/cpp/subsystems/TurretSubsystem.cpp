// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"

TurretSubsystem::TurretSubsystem() {
    frc::SmartDashboard::PutNumber("kp_hAim", kp_hAim);
    frc::SmartDashboard::PutNumber("kp_vAimty", kp_vAimty);
    frc::SmartDashboard::PutNumber("kp_vAimre", kp_vAimre);
    m_vTurretMotorLeft.SetOpenLoopRampRate(0.2);
    m_vTurretMotorRight.SetOpenLoopRampRate(0.2);

    //m_vTurretMotorLeft.Follow(m_vTurretMotorLeft);
}
void TurretSubsystem::setSpeed(float speed) {
    // m_turretMotor.Set(speed);
}
// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {

    kp_hAim = frc::SmartDashboard::GetNumber("kp_hAim", kp_hAim); //Horizontal Aim
    kp_vAimty = frc::SmartDashboard::GetNumber("kp_vAimty", kp_vAimty); //Vertical Aim
    kp_vAimre = frc::SmartDashboard::GetNumber("kp_vAimre", kp_vAimre); //Vertical Aim


    frc::SmartDashboard::PutBoolean("Right Limit switch for vert shooter, ", ls_vTurretMotorRight.Get());
    frc::SmartDashboard::PutBoolean("Left Limit switch for vert shooter, ", ls_vTurretMotorLeft.Get());

    frc::SmartDashboard::PutNumber("Right Encoder for vert shooter, ", re_vTurretMotorRight.GetPosition());
    frc::SmartDashboard::PutNumber("Left Encoder for vert shooter, ", re_vTurretMotorLeft.GetPosition());

    frc::SmartDashboard::PutNumber("TurretState", TurretState);

    if(TurretState == R_BOTH){
        m_vTurretMotorRight.Set(0.2);
        m_vTurretMotorLeft.Set(0.2);

        if(ls_vTurretMotorRight.Get() == true) { 
            TurretState = R_LEFT;
            re_vTurretMotorRight.SetPosition(0);
        } 
        if(ls_vTurretMotorLeft.Get() == true) { 
            TurretState = R_RIGHT;
            re_vTurretMotorLeft.SetPosition(0);
        } 

    } else if(TurretState == R_LEFT){
        m_vTurretMotorRight.Set(0.0);
        m_vTurretMotorLeft.Set(0.2);

        if(ls_vTurretMotorLeft.Get() == true) { 
            TurretState = PRESHOOT_RAISE; 
            re_vTurretMotorLeft.SetPosition(0);
        } 
        
    } else if(TurretState == R_RIGHT){
        m_vTurretMotorRight.Set(0.2);
        m_vTurretMotorLeft.Set(0.0);

        if(ls_vTurretMotorRight.Get() == true) { 
            TurretState = PRESHOOT_RAISE;
            re_vTurretMotorRight.SetPosition(0);
        }
    } else if (TurretState == PRESHOOT_RAISE){
        
        if(re_vTurretMotorRight.GetPosition() > -400) {
            m_vTurretMotorRight.Set(-1.0);
            m_vTurretMotorLeft.Set(-1.0);
        } else {
            m_vTurretMotorRight.Set(0.0);
            m_vTurretMotorLeft.Set(0.0);
            TurretState = DRIVER_SHOOT;
        }

    } else if (TurretState == DRIVER_SHOOT){

        m_vTurretMotorRight.Set(cur_stickValV); 
        m_vTurretMotorLeft.Set(cur_stickValV); 
        
        m_hTurretMotor.Set(cur_stickValH); 

        //m_VerturretMotor.Set(m_xBox->GetRawAxis(1));
    } else if (TurretState == AUTO_SHOOT){
        //ledMode 
        // 3 on 
        // 0 off 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("ledMode", 3); 
        float tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
        float ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
        re_vTurretMotorRight.GetPosition();
        m_hTurretMotor.Set(tx * kp_hAim);

        if (true){
            m_vTurretMotorRight.Set(ty * kp_vAimty);
            m_vTurretMotorLeft.Set(ty * kp_vAimty);

        } else {
            m_vTurretMotorRight.Set((re_vTurretMotorRight.GetPosition() - (-700)) * kp_vAimre);
            m_vTurretMotorLeft.Set((re_vTurretMotorLeft.GetPosition() - (-700)) * kp_vAimre);

        }   

    }

    
    if (cur_stickPOV == 0){
        frc::SmartDashboard::PutNumber("Set RPM 2", 4000);
    } else if (cur_stickPOV == 90){
        frc::SmartDashboard::PutNumber("Set RPM 2", 4500);
    } else if (cur_stickPOV == 180){
        frc::SmartDashboard::PutNumber("Set RPM 2", 5200);
    } else if (cur_stickPOV == 270){
        frc::SmartDashboard::PutNumber("Set RPM 2", 3700);
    } 
    
}

void TurretSubsystem::setAngleV(float l_stickValV) {
    //std::cout << "setAngleV run, ";

    // Generate Deadzone
    // double deadzone = 0.15;
    if(fabs(l_stickValV) > 0.15) {
        cur_stickValV = l_stickValV;
    } else {
        cur_stickValV = 0.0;
    }

}

void TurretSubsystem::setAngleH(float l_stickValH) {
    //std::cout << "setAngleH val " << l_stickValH << ",";

    // Generate Deadzone with Offset-Shift
    if(l_stickValH > 0.15) {
        cur_stickValH = 0.4*(l_stickValH - 0.15); //0.4 was 0.25
    } else if (l_stickValH < -0.15 ) {
        cur_stickValH = 0.4*(l_stickValH + 0.15);
    } else {
        cur_stickValH = 0.0;
    }

    //std::cout << "setAngleH val " << cur_stickValH << ",";

}

void TurretSubsystem::setAutoAimOn() {
    TurretState = AUTO_SHOOT;
}

void TurretSubsystem::setManuelAimOn() {
    TurretState = DRIVER_SHOOT;
}

void TurretSubsystem::setStickPOV(int stickPOV){
    cur_stickPOV = stickPOV;
}