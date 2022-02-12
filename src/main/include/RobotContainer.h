// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include "subsystems/DriveTrain.h"
#include "commands/JetsonDrive.h" 
#include <iostream>
// #include <frc/Joystick.h>
// #include "rev/CANSparkMax.h"
// #include "commands/ExampleCommand.h"
// #include "subsystems/ExampleSubsystem.h"


//random stuff

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

   frc2::Command* GetAutonomousCommand();

 private:
  // The robot's subsystems and commands are defined here...
  DriveTrain m_drivetrain;
  // frc::Joystick stick1{1};
  // frc::Joystick stick2{2};
 // ExampleSubsystem m_subsystem;
  JetsonDrive m_autonomousCommand{&m_drivetrain};


  void ConfigureButtonBindings();
};
