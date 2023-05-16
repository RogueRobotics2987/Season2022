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

std::string TurretSubsystem::GetLog() {
    
    std::string ret_string = "TurretState: " + std::to_string(TurretState) + "\n" + 
    "cur_stickValV: " + std::to_string(cur_stickValV) + "\n" + 
    "cur_stickValH: " + std::to_string(cur_stickValH) + "\n" + 
    "kp_hAim: " + std::to_string(kp_hAim) + "\n" + 
    "kp_vAimty: " + std::to_string(kp_vAimty) + "\n" + 
    "kp_vAimre: " + std::to_string(kp_vAimre) + "\n" + 
    "turretScaleVal: " + std::to_string(turretScaleVal) + "\n" + 
    // "cur_pipeline: " + std::to_string(cur_pipeline) + "\n" + 
    "cur_stickPOV: " + std::to_string(cur_stickPOV) + "\n";
    //"m_onTarget: " + std::to_string(m_onTarget) + "\n";
    // enum TurretState_t {INIT, R_CENTER, R_BOTH, R_LEFT, R_RIGHT, DRIVER_SHOOT, AUTO_SHOOT, PRESHOOT_RAISE, VERT_AIM}; //R_ means retract
    // TurretState_t TurretState = R_CENTER;

    // double cur_stickValV = 0.0;
    // double cur_stickValH = 0.0;
    // double kp_hAim = 0.01;
    // double kp_vAimty = 1.0;
    // double kp_vAimre = 0.02;
    // double turretScaleVal = 0.4; //percent of max speed 

    // int cur_pipeline = 0;
    // int cur_stickPOV = 0;
    // // bool m_onTarget = false;
    // // Components (e.g. motor controllers and sensors) should generally be
    // // declared private and exposed only through public methods.


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

    //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", cur_pipeline);


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

        // float tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
        // float ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);

        m_vTurretMotorCenter.Set(-cur_stickValV); 
        // m_vTurretMotorRight.Set(cur_stickValV); 
        // m_vTurretMotorLeft.Set(cur_stickValV); 

    //     if(-1.0 < tx && tx < 1.0 && -1.0 < ty && ty < 1.0) {
    //         frc::SmartDashboard::PutBoolean("Turret Manual Target Locked", true);
    // }
    //     else {
    //         frc::SmartDashboard::PutBoolean("Turret Manual Target Locked", false);
    //     }

        m_hTurretMotor.Set(cur_stickValH); 

        //m_VerturretMotor.Set(m_xBox->GetRawAxis(1));
    } else if (TurretState == AUTO_SHOOT){
        //ledMode 
        // 3 on 
        // 0 off 
        //cur_pipeline = nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr") -> GetNumber("pipeline", cur_pipeline);
        /*float tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
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
    */
    } else if (TurretState == VERT_AIM){
        m_vTurretMotorCenter.Set(-1.0 * (re_vTurretMotorCenter.GetPosition() - (29)) * kp_vAimre);
    }   

    
    if (cur_stickPOV == 0){
        //default settings
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 2950);//was 4000 //was 2500
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 2950);//was 4000
        //cur_pipeline = 0; in Sam's code
        //cur_pipeline = 0;
        //Mura close settings 7
    } else if (cur_stickPOV == 90){
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 2900);//was 3100
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 2900);//was 3450
        //cur_pipeline = 0;
        //cur_pipeline = 6;
        //Mura launch pad 6

    } else if (cur_stickPOV == 180){
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 3000);
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 3000);
        //cur_pipeline =0;
        //cur_pipeline = 4;
        //Mura human player spot 4

    } else if (cur_stickPOV == 270){
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 3500);
        frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 3500);
        //cur_pipeline = 1;
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

void TurretSubsystem::setManualAimOn() {
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
