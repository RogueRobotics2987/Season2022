// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/intake.h"

intake::intake() = default;

// This method will be called once per scheduler run
void intake::Periodic() {}
    void intake::intakeBall (double setVal) {
        p_intakeMotor->Set (setVal);
    }
    void intake::StartConveyor (double percent) {
        p_conveyorMotor-> Set (percent);
    }
    void intake::StopMotors () {
        p_intakeMotor->Set (0);
    }