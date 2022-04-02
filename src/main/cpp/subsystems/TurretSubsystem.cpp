// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"

TurretSubsystem::TurretSubsystem() {
    frc::SmartDashboard::PutNumber("kp_hAim", kp_hAim);
    frc::SmartDashboard::PutNumber("kp_vAimty", kp_vAimty);
    frc::SmartDashboard::PutNumber("kp_vAimre", kp_vAimre);
    // m_vTurretMotorLeft.SetOpenLoopRampRate(0.2);
    // m_vTurretMotorRight.SetOpenLoopRampRate(0.2);
    m_vTurretMotorCenter.SetOpenLoopRampRate(0.2);

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


    frc::SmartDashboard::PutBoolean("Turret Center Limit Switch Vert", ls_vTurretMotorCenter.Get());
    // frc::SmartDashboard::PutBoolean("Right Limit switch for vert shooter, ", ls_vTurretMotorRight.Get());
    // frc::SmartDashboard::PutBoolean("Left Limit switch for vert shooter, ", ls_vTurretMotorLeft.Get());
    frc::SmartDashboard::PutBoolean("Turret Manual Target Locked", false);

    frc::SmartDashboard::PutNumber("Turret Vert Encoder", re_vTurretMotorCenter.GetPosition());
    // frc::SmartDashboard::PutNumber("Right Encoder for vert shooter, ", re_vTurretMotorRight.GetPosition());
    // frc::SmartDashboard::PutNumber("Left Encoder for vert shooter, ", re_vTurretMotorLeft.GetPosition());

    frc::SmartDashboard::PutNumber("TurretState", TurretState);
    frc::SmartDashboard::PutNumber("TurretScaleVal", turretScaleVal);

    //nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("pipeline", cur_pipeline);


    frc::SmartDashboard::PutNumber("Turret H Position", re_hTurretMotor.GetPosition());

    if(TurretState == R_BOTH){
        // m_vTurretMotorRight.Set(0.2);
        // m_vTurretMotorLeft.Set(0.2);

        // if(ls_vTurretMotorRight.Get() == true) { 
        //     TurretState = R_LEFT;
        //     re_vTurretMotorRight.SetPosition(0);
        // } 
        // if(ls_vTurretMotorLeft.Get() == true) { 
        //     TurretState = R_RIGHT;
        //     re_vTurretMotorLeft.SetPosition(0);
        // } 

    } else if(TurretState == R_CENTER){
        m_vTurretMotorCenter.Set(-0.2);

        if(ls_vTurretMotorCenter.Get() == true) { 
            TurretState = PRESHOOT_RAISE; 
            re_vTurretMotorCenter.SetPosition(0);
        } 
        
    } else if(TurretState == R_LEFT){
        // m_vTurretMotorRight.Set(0.0);
        // m_vTurretMotorLeft.Set(0.2);

        // if(ls_vTurretMotorLeft.Get() == true) { 
        //     TurretState = PRESHOOT_RAISE; 
        //     re_vTurretMotorLeft.SetPosition(0);
        // } 
        
    } else if(TurretState == R_RIGHT){
        // m_vTurretMotorRight.Set(0.2);
        // m_vTurretMotorLeft.Set(0.0);

        // if(ls_vTurretMotorRight.Get() == true) { 
        //     TurretState = PRESHOOT_RAISE;
        //     re_vTurretMotorRight.SetPosition(0);
        // }
    } else if (TurretState == PRESHOOT_RAISE){
        
        if(re_vTurretMotorCenter.GetPosition() < 29) {//was -400 //was -200
        //will need to set the speed back to 0.5
            m_vTurretMotorCenter.Set(0.5); //might be inverted
            // m_vTurretMotorRight.Set(-1.0);
            // m_vTurretMotorLeft.Set(-1.0);
        } else {
            m_vTurretMotorCenter.Set(0.0);
            // m_vTurretMotorRight.Set(0.0);
            // m_vTurretMotorLeft.Set(0.0);
            TurretState = DRIVER_SHOOT;
        }

    } else if (TurretState == DRIVER_SHOOT){

        float tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
        float ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);

        m_vTurretMotorCenter.Set(-cur_stickValV); 
        // m_vTurretMotorRight.Set(cur_stickValV); 
        // m_vTurretMotorLeft.Set(cur_stickValV); 

        if(-1.0 < tx && tx < 1.0 && -1.0 < ty && ty < 1.0) {
            frc::SmartDashboard::PutBoolean("Turret Manual Target Locked", true);
    }
        else {
            frc::SmartDashboard::PutBoolean("Turret Manual Target Locked", false);
        }

        m_hTurretMotor.Set(cur_stickValH); 

        //m_VerturretMotor.Set(m_xBox->GetRawAxis(1));
    } else if (TurretState == AUTO_SHOOT){
        //ledMode 
        // 3 on 
        // 0 off 
        //cur_pipeline = nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr") -> GetNumber("pipeline", cur_pipeline);
        float tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
        float ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);

        // re_vTurretMotorRight.GetPosition();  // Orser removed 3/26, shouldn't do anything?
        m_hTurretMotor.Set(tx * kp_hAim);

        if(-1.0 < tx && tx < 1.0 && -1.0 < ty && ty < 1.0) {
            frc::SmartDashboard::PutBoolean("Turret Manual Target Locked", true);
        }
        else {
            frc::SmartDashboard::PutBoolean("Turret Manual Target Locked", false);
        }

        if (true){
            m_vTurretMotorCenter.Set(-1.0 * ty * kp_vAimty);
            // m_vTurretMotorRight.Set(ty * kp_vAimty);
            // m_vTurretMotorLeft.Set(ty * kp_vAimty);

        } else {
            m_vTurretMotorCenter.Set(-1.0 * (re_vTurretMotorCenter.GetPosition() - (-700)) * kp_vAimre);
            // m_vTurretMotorRight.Set((re_vTurretMotorRight.GetPosition() - (-700)) * kp_vAimre);
            // m_vTurretMotorLeft.Set((re_vTurretMotorLeft.GetPosition() - (-700)) * kp_vAimre);

        }   

    } else if (TurretState == VERT_AIM){
        m_vTurretMotorCenter.Set(-1.0 * (re_vTurretMotorCenter.GetPosition() - (29)) * kp_vAimre);
    }

    
    if (cur_stickPOV == 0){
        //default settings
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 2900);//was 4000 //was 2500
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 2900);//was 4000
        //cur_pipeline = 0; in Sam's code
        cur_pipeline = 7;//Mura close settings
    } else if (cur_stickPOV == 90){
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 3450);//was 3100
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 3450);
        cur_pipeline = 6;//Mura launch pad

    } else if (cur_stickPOV == 180){
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 3900);
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 3900);
        cur_pipeline = 4;//Mura human player spot

    } else if (cur_stickPOV == 270){
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 800);
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 800);
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
    frc::SmartDashboard::GetNumber("TurretScaleVal", turretScaleVal);
    // Generate Deadzone with Offset-Shift
    if(l_stickValH > 0.15) {
        cur_stickValH = turretScaleVal*(l_stickValH - 0.15); //0.4 was 0.25
    } else if (l_stickValH < -0.15 ) {
        cur_stickValH = turretScaleVal*(l_stickValH + 0.15);
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

double TurretSubsystem::getHPosition() {
    return re_hTurretMotor.GetPosition();
}

void TurretSubsystem::setVertAimOn() {
    TurretState = VERT_AIM;
}
/*void TurretSubsystem::setLowGoalAim(){
    if(re_vTurretMotorCenter.GetPosition() < 75) {//was -400 //was -200
            m_vTurretMotorCenter.Set(0.2); 
        } else if(re_vTurretMotorCenter.GetPosition() > 75) {
            m_vTurretMotorCenter.Set(-0.2);
        }
}*/