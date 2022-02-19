// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"

TurretSubsystem::TurretSubsystem() {
    frc::SmartDashboard::PutNumber("kp_hAim", kp_hAim);

    //Turret = rev::CANSparkMax(60, rev::CANSparkMax::MotorType::kBrushless);
    // Turret(60, rev::CANSparkMax::MotorType::kBrushless);
}
void TurretSubsystem::setSpeed(float speed) {
    // m_turretMotor.Set(speed);
}
// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {

    kp_hAim = frc::SmartDashboard::GetNumber("kp_hAim", kp_hAim);

    frc::SmartDashboard::PutBoolean("Limit switch for vert shooter, ", ls_vTurretMotor.Get());
    frc::SmartDashboard::PutNumber("Encoder for vert shooter, ", re_vTurretMotor.GetPosition());


    if(actuatorState == 0){
        std::cout << "TurretSubSysPeriod:0," << cur_stickValV << "," << std::endl;
        m_vTurretMotor.Set(0.4);
        // Temp disable statemachine...
         if(ls_vTurretMotor.Get() == true) { //(ls_turret.Get() == false){
            actuatorState = 1;
            re_vTurretMotor.SetPosition(0);

        }
    }
    else if (actuatorState == 1){
        m_vTurretMotor.Set(cur_stickValV);
        m_hTurretMotor.Set(cur_stickValH);

        //m_VerturretMotor.Set(m_xBox->GetRawAxis(1));

    }
    else if (actuatorState == 2){
        //ledMode 
        // 3 on 
        // 0 off 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("ledMode", 3); 

        float tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);

        m_hTurretMotor.Set(tx * kp_hAim);


    }
}

void TurretSubsystem::setAngleV(float l_stickValV) {
    std::cout << "setAngleV run, ";

    // Generate Deadzone
    // double deadzone = 0.15;
    if(fabs(l_stickValV) > 0.15) {
        cur_stickValV = l_stickValV;
    } else {
        cur_stickValV = 0.0;
    }

}

void TurretSubsystem::setAngleH(float l_stickValH) {
    std::cout << "setAngleH val " << l_stickValH << ",";

    // Generate Deadzone with Offset-Shift
    if(l_stickValH > 0.15) {
        cur_stickValH = 0.25*(l_stickValH - 0.15);
    } else if (l_stickValH < -0.15 ) {
        cur_stickValH = 0.25*(l_stickValH + 0.15);
    } else {
        cur_stickValH = 0.0;
    }

    std::cout << "setAngleH val " << cur_stickValH << ",";

}

void TurretSubsystem::setAutoAimOn() {
    actuatorState = 2;
}

void TurretSubsystem::setManuelAimOn() {
    actuatorState = 1;
}