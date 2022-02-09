// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem){
  // Initialize all of your commands and subsystems here
  drivetrain.SetDefaultCommand(TankDrive(drivetrain, stick1, stick2));
  intake.SetDefaultCommand(PickUpBall(intake, xbox, stick2));
  // Configure the button bm_indings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  frc2::JoystickButton(&xbox, 4).WhenHeld(&m_intakeIn); 
  frc2::JoystickButton(&xbox, 2).WhenHeld(&m_intakeOut); 
  frc2::JoystickButton(&xbox, 1).WhenHeld(&m_conveyerForward); 
  frc2::JoystickButton(&stick2, 2).WhenHeld(&m_conveyerBackward); 

}
frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand; 
}

