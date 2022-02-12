// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer(){
  // std::cout<<"Robot container constructor"<<std::endl;


  // Initialize all of your commands and subsystems here
 //m_drivetrain.SetDefaultCommand(JetsonDrive(&m_drivetrain));
  // Configure the button bm_indings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}
 frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand; 
 }

