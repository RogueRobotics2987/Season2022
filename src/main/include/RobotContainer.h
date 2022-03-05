// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include "subsystems/DriveTrain.h"
#include "commands/TankDrive.h" 
#include <frc/Joystick.h>
#include "rev/CANSparkMax.h"
#include "subsystems/Intake.h"
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include "commands/Auto.h"
#include "subsystems/Shooter.h"
#include "subsystems/Climber.h"
#include "commands/ClimbCmd.h"
#include "subsystems/TurretSubsystem.h"
#include "commands/TurretCmd.h" 
#include "commands/AimFor_T.h"
#include "commands/TimerCMD.h"
#include "commands/SafeBallShoot.h"

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

//Sam/Corey(?) stuff
#include "commands/AimAtTarget.h"
#include "commands/LimelightSingleTarget.h"
#include "commands/LimelightTriTarget.h"

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

 private:
  // The robot's subsystems and commands are defined here...
  DriveTrain drivetrain;
  Shooter m_shooter;
  Intake intake;
  TurretSubsystem m_turret;
  Climber climber;

  frc::Joystick xbox{0};
  frc::Joystick stick1{1};
  frc::Joystick stick2{2};

  void ConfigureButtonBindings();

  frc2::InstantCommand m_conveyerForward{[this] {intake.ConveyorForward();}, {&intake}};
  frc2::InstantCommand m_conveyerForwardRelease{[this] {intake.ConveyorForwardRelease();}, {&intake}};

  frc2::InstantCommand m_conveyerBackward{[this] {intake.ConveyorBackward();}, {&intake}};
  frc2::InstantCommand m_conveyerBackwardRelease{[this] {intake.ConveyorBackwardRelease();}, {&intake}};

  frc2::InstantCommand m_intakeIn{[this] {intake.IntakeIn();}, {&intake}};
  frc2::InstantCommand m_intakeInRelease{[this] {intake.IntakeInRelease();}, {&intake}};

  frc2::InstantCommand m_intakeOut{[this] {intake.IntakeOut();}, {&intake}};
  frc2::InstantCommand m_intakeOutRelease{[this] {intake.IntakeOutRelease();}, {&intake}};

  frc2::InstantCommand m_shooter2000{[this] {m_shooter.setShooter();}, {&m_shooter}};
  frc2::InstantCommand m_shooterStop{[this] {m_shooter.stopShooter();}, {&m_shooter}};

  frc2::InstantCommand m_TurtModeAuto{[this] {m_turret.setAutoAimOn();}, {&m_turret}};
  frc2::InstantCommand m_TurtModeManu{[this] {m_turret.setManuelAimOn();}, {&m_turret}};

  // frc2::InstantCommand m_dropIntake{[this] {m_turret.setManuelAimOn();}, {&m_turret}};

  frc2::Command* m_autonomousCommand;

  frc::Trajectory turn180;
  frc::Trajectory twoBall1_1;
  frc::Trajectory twoBall1_2;
  frc::Trajectory twoBall1_3;

};
