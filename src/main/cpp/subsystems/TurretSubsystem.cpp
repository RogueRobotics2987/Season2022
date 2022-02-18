// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"

TurretSubsystem::TurretSubsystem() {
    //Turret = rev::CANSparkMax(60, rev::CANSparkMax::MotorType::kBrushless);
    // Turret(60, rev::CANSparkMax::MotorType::kBrushless);
}
void TurretSubsystem::setSpeed(float speed) {
    // m_vertMotor.Set(speed);
}
// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {



m_vertMotor.Set(0.2);
      
    
    if (state <= 0){

        std::cout << "Turret state = " << state << std::endl;


        ActuatorSpeed = 0.2;
        m_vertMotor.Set(ActuatorSpeed);
        if(ls_vertMotor.Get() == false){
            state = 1;

        }
    }
    else if (state == 1){
        ActuatorSpeed = 0.0;
        m_vertMotor.Set(ActuatorSpeed);
    }
}
