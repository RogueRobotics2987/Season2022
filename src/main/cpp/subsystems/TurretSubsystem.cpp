// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"

TurretSubsystem::TurretSubsystem() {
    //Turret = rev::CANSparkMax(60, rev::CANSparkMax::MotorType::kBrushless);
    // Turret(60, rev::CANSparkMax::MotorType::kBrushless);
}
void TurretSubsystem::setSpeed(float speed) {
    // m_turretMotor.Set(speed);
}
// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {

    if(actuatorState == 0){
        m_vTurretMotor.Set(cur_stickValV);
        std::cout << "TurretSubSysPeriod:0," << cur_stickValV << "," << std::endl;

        // Temp disable statemachine...
         if(false) { //(ls_turret.Get() == false){
            actuatorState = 1;
        }
    }
    else if (actuatorState == 1){
        //m_VerturretMotor.Set(m_xBox->GetRawAxis(1));

    }
}

void TurretSubsystem::setAngleV(float l_stickValV) {
    std::cout << "setAngleV run, ";
    if(fabs(l_stickValV) > 0.1) {
        cur_stickValV = l_stickValV;
    } else {
        cur_stickValV = 0.0;
    }

}

