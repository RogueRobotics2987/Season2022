// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {

  // Initialize all of your commands and subsystems here
  drivetrain.SetDefaultCommand(TankDrive(drivetrain, stick1, stick2));
  m_turret.SetDefaultCommand(TurretCmd(m_turret, stick1, stick2, xbox));
  climber.SetDefaultCommand(ClimbCmd(climber, xbox, stick1, stick2));

  // Configure the button bindings
  ConfigureButtonBindings();
  
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  frc2::JoystickButton(&stick2, 1).WhenPressed(&m_intakeIn); //wwas xbox 4
  frc2::JoystickButton(&stick2, 1).WhenReleased(&m_intakeInRelease); 

  frc2::JoystickButton(&stick1, 1).WhenPressed(&m_intakeOut); //was xbox 2
  frc2::JoystickButton(&stick1, 1).WhenReleased(&m_intakeOutRelease); 

  frc2::JoystickButton(&xbox, 3).WhenPressed(&m_conveyerForward); //was xbox 1
  frc2::JoystickButton(&xbox, 3).WhenReleased(&m_conveyerForwardRelease); 

  frc2::JoystickButton(&xbox, 1).WhenPressed(&m_conveyerBackward); //was stick2 2
  frc2::JoystickButton(&xbox, 1).WhenReleased(&m_conveyerBackwardRelease); 

  // frc2::JoystickButton(&xbox, 3).WhenPressed(&m_shooter2000); 
  // frc2::JoystickButton(&xbox, 3).WhenReleased(&m_shooterStop); 

  frc2::JoystickButton(&xbox, 6).WhenPressed(&m_shooter2000); //right bumper
  frc2::JoystickButton(&xbox, 5).WhenPressed(&m_shooterStop); //left bumper
  
  frc2::JoystickButton(&xbox, 7).WhenPressed(&m_TurtModeAuto); 
  frc2::JoystickButton(&xbox, 7).WhenReleased(&m_TurtModeManu); 

  
}


frc2::Command* RobotContainer::GetAutonomousCommand() {


  // frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
  //     frc::SimpleMotorFeedforward<units::meters>(
  //     DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
  //     DriveConstants::kDriveKinematics, 10_V);

  frc::TrajectoryConfig config{AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}; 
  config.SetKinematics(DriveConstants::kDriveKinematics);
  // config.AddConstraint(autoVoltageConstraint);

  std::string startGameFile = frc::filesystem::GetDeployDirectory() + "/paths/startGame.wpilib.json";
    // wpi::sys::path::append(startGameFile, "paths/startGame.wpilib.json");
    frc::Trajectory startGame = frc::TrajectoryUtil::FromPathweaverJson(startGameFile);

  std::string circleFile = frc::filesystem::GetDeployDirectory() + "/paths/Circle.wpilib.json";
    frc::Trajectory circle = frc::TrajectoryUtil::FromPathweaverJson(circleFile);

auto trajectoryOne = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      //just go straight forward
      // {frc::Translation2d(1_m, 0_m)},

     {frc::Translation2d(1.0_m, 1.0_m), 
      frc::Translation2d(2.0_m, 2.0_m)},

      frc::Pose2d(3_m, 2_m, frc::Rotation2d(0_deg)),

      // Pass the config 
      config
);

// First Test
frc2::RamseteCommand ramseteCommandTrajectoryOne(
      trajectoryOne, [this]() { return drivetrain.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return drivetrain.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
      {&drivetrain});

// First Pathweaver Test
frc2::RamseteCommand ramseteCommandStartGame(
      startGame, [this]() { return drivetrain.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return drivetrain.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
      {&drivetrain});

// Circle around the arena
frc2::RamseteCommand ramseteCommandCircle(
      circle, [this]() { return drivetrain.GetPose(); },
      frc::RamseteController(AutoConstants::kRamseteB,
                             AutoConstants::kRamseteZeta),
      frc::SimpleMotorFeedforward<units::meters>(
          DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
      DriveConstants::kDriveKinematics,
      [this] { return drivetrain.GetWheelSpeeds(); },
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
      [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
      {&drivetrain});


        // drivetrain.ResetOdometry(trajectoryOne.InitialPose());
        // drivetrain.ResetOdometry(startGame.InitialPose());
        drivetrain.ResetOdometry(circle.InitialPose());




      frc2::SequentialCommandGroup* trajectoryOneGroup = new frc2::SequentialCommandGroup(
        std::move(ramseteCommandTrajectoryOne),
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {})
        // Auto Aim
        //Shooter
      );
      frc2::SequentialCommandGroup* startGameGroup = new frc2::SequentialCommandGroup(
        std::move(ramseteCommandStartGame),
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      );
      frc2::SequentialCommandGroup* circleGroup = new frc2::SequentialCommandGroup(
        std::move(ramseteCommandCircle),
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      );



  // An example command will be run in autonomous
      // return trajectoryOneGroup;
      // return startGameGroup;
      return circleGroup;

  // return &m_autonomousCommand;

//  return Auto(drivetrain, 1.0, -4.0);
}

