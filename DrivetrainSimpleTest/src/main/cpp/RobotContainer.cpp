// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/Joystick.h>
#include "subsystems/DriveTrain.h"
#include "commands/TankDrive.h"
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>



RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem/*&m_drivetrain*/) {
  // Initialize all of your commands and subsystems here

  m_drivetrain.Log(); 
  m_drivetrain.SetDefaultCommand(TankDrive(&m_drivetrain, &joy1));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
