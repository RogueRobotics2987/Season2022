// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AutoAim.h"

double AutoAim::GetTX(){
    return tx; 
}
double AutoAim::GetTY(){
    return ty; 
}


AutoAim::AutoAim() {
  // Implementation of subsystem constructor goes here.
}

void AutoAim::Periodic() {

    tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx",0.0);
    ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);

    // must be moved to command 
    // tx = limelightTable->GetNumber("tx", 0.0); 
    // ty = limelightTable->GetNumber("ty", 0.0); 




    
    //stays in periodic
    frc::SmartDashboard::PutNumber("Limelight X", tx);
    frc::SmartDashboard::PutNumber("Limelight Y", ty);
    
    
    }

void AutoAim::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
