// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/smartdashboard/SmartDashboard.h>

class AutoAim : public frc2::SubsystemBase {
 public:
  AutoAim();

 
  void Periodic();
  
  
  double GetTY(); 
  double GetTX(); 
  void SimulationPeriodic();


 private:
   std::shared_ptr<nt::NetworkTable> limelightTable;
   
   float tx = 0;
   float ty = 0;
   

   
};
