// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include "subsystems/Climber.h"

#include "commands/ServoTest.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/controller/PIDController.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/controller/RamseteController.h>
#include <wpi/SmallString.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>


#include <cameraserver/CameraServer.h>

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
  frc2::Command* GetCloseBallAuto();
  frc2::Command* GetThreeBallAuto();
  frc2::Command* GetTwoBallAuto();
  frc2::Command* GetLimelightLockOn();

 private:
  // The robot's subsystems and commands are defined here...
  Climber climber;

  frc::Joystick xbox{0};
  frc::Joystick stick1{1};
  frc::Joystick stick2{2};

};
