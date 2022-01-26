// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/TurretSubsystem.h"

TurretSubsystem::TurretSubsystem() {
    //Turret = rev::CANSparkMax(60, rev::CANSparkMax::MotorType::kBrushless);
    // Turret(60, rev::CANSparkMax::MotorType::kBrushless);
}
void TurretSubsystem::setSpeed(float speed) {
    // m_turret.Set(speed);
}
// This method will be called once per scheduler run
void TurretSubsystem::Periodic() {}
