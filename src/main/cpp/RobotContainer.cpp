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


frc2::Command* RobotContainer::GetCloseBallAuto() {


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

  // std::string circleFile = frc::filesystem::GetDeployDirectory() + "/paths/Circle.wpilib.json";
  //   frc::Trajectory circle = frc::TrajectoryUtil::FromPathweaverJson(circleFile);

  // std::string bluePosition3File = frc::filesystem::GetDeployDirectory() + "/paths/BluePosition3.wpilib.json";
  //   frc::Trajectory bluePosition3 = frc::TrajectoryUtil::FromPathweaverJson(bluePosition3File);
  
  // std::string bluePosition2File = frc::filesystem::GetDeployDirectory() + "/paths/BluePosition2.wpilib.json";
  //   frc::Trajectory bluePosition2 = frc::TrajectoryUtil::FromPathweaverJson(bluePosition2File);

  // std::string bluePosition2_1File = frc::filesystem::GetDeployDirectory() + "/paths/BluePosition2.1.wpilib.json";
  //   frc::Trajectory bluePosition2_1 = frc::TrajectoryUtil::FromPathweaverJson(bluePosition2_1File);

  // std::string bluePosition2_2File = frc::filesystem::GetDeployDirectory() + "/paths/BluePosition2.2.wpilib.json";
  //   frc::Trajectory bluePosition2_2 = frc::TrajectoryUtil::FromPathweaverJson(bluePosition2_2File);

  // std::string bluePosition1File = frc::filesystem::GetDeployDirectory() + "/paths/BluePosition1.wpilib.json";
  //   frc::Trajectory bluePosition1 = frc::TrajectoryUtil::FromPathweaverJson(bluePosition1File);

  

  std::string turn180File = frc::filesystem::GetDeployDirectory() + "/paths/Turn180.wpilib.json";
  turn180 = frc::TrajectoryUtil::FromPathweaverJson(turn180File);





// auto trajectoryOne = frc::TrajectoryGenerator::GenerateTrajectory(
//       // Start at the origin facing the +X direction
//       frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
//       //just go straight forward
//       // {frc::Translation2d(1_m, 0_m)},

//      {frc::Translation2d(1.0_m, 1.0_m), 
//       frc::Translation2d(2.0_m, 2.0_m)},

//       frc::Pose2d(3_m, 2_m, frc::Rotation2d(0_deg)),

//       // Pass the config 
//       config
// );


// First Test
// frc2::RamseteCommand ramseteCommandTrajectoryOne(
//       trajectoryOne, [this]() { return drivetrain.GetPose(); },
//       frc::RamseteController(AutoConstants::kRamseteB,
//                              AutoConstants::kRamseteZeta),
//       frc::SimpleMotorFeedforward<units::meters>(
//           DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
//       DriveConstants::kDriveKinematics,
//       [this] { return drivetrain.GetWheelSpeeds(); },
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
//       {&drivetrain});

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
// frc2::RamseteCommand ramseteCommandCircle(
//       circle, [this]() { return drivetrain.GetPose(); },
//       frc::RamseteController(AutoConstants::kRamseteB,
//                              AutoConstants::kRamseteZeta),
//       frc::SimpleMotorFeedforward<units::meters>(
//           DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
//       DriveConstants::kDriveKinematics,
//       [this] { return drivetrain.GetWheelSpeeds(); },
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
//       {&drivetrain});

// frc2::RamseteCommand ramseteCommandBluePosition3(
//       bluePosition3, [this]() { return drivetrain.GetPose(); },
//       frc::RamseteController(AutoConstants::kRamseteB,
//                              AutoConstants::kRamseteZeta),
//       frc::SimpleMotorFeedforward<units::meters>(
//           DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
//       DriveConstants::kDriveKinematics,
//       [this] { return drivetrain.GetWheelSpeeds(); },
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
//       {&drivetrain});

frc2::RamseteCommand ramseteCommandTurn180(
      turn180, [this]() { return drivetrain.GetPose(); },
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

// // Blue Positions
// frc2::RamseteCommand ramseteCommandBluePosition2(
//       bluePosition2, [this]() { return drivetrain.GetPose(); },
//       frc::RamseteController(AutoConstants::kRamseteB,
//                              AutoConstants::kRamseteZeta),
//       frc::SimpleMotorFeedforward<units::meters>(
//           DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
//       DriveConstants::kDriveKinematics,
//       [this] { return drivetrain.GetWheelSpeeds(); },
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
//       {&drivetrain});

// frc2::RamseteCommand ramseteCommandBluePosition2_1(
//       bluePosition2_1, [this]() { return drivetrain.GetPose(); },
//       frc::RamseteController(AutoConstants::kRamseteB,
//                              AutoConstants::kRamseteZeta),
//       frc::SimpleMotorFeedforward<units::meters>(
//           DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
//       DriveConstants::kDriveKinematics,
//       [this] { return drivetrain.GetWheelSpeeds(); },
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
//       {&drivetrain});

// frc2::RamseteCommand ramseteCommandBluePosition2_2(
//       bluePosition2_2, [this]() { return drivetrain.GetPose(); },
//       frc::RamseteController(AutoConstants::kRamseteB,
//                              AutoConstants::kRamseteZeta),
//       frc::SimpleMotorFeedforward<units::meters>(
//           DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
//       DriveConstants::kDriveKinematics,
//       [this] { return drivetrain.GetWheelSpeeds(); },
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
//       {&drivetrain});


// frc2::RamseteCommand ramseteCommandBluePosition1(
//       bluePosition2_2, [this]() { return drivetrain.GetPose(); },
//       frc::RamseteController(AutoConstants::kRamseteB,
//                              AutoConstants::kRamseteZeta),
//       frc::SimpleMotorFeedforward<units::meters>(
//           DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
//       DriveConstants::kDriveKinematics,
//       [this] { return drivetrain.GetWheelSpeeds(); },
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       frc2::PIDController(DriveConstants::kPDriveVel, 0, 0),
//       [this](auto left, auto right) { drivetrain.TankDriveVolts(left, right); },
//       {&drivetrain});






        // drivetrain.ResetOdometry(trajectoryOne.InitialPose());
        // drivetrain.ResetOdometry(startGame.InitialPose());
        // drivetrain.ResetOdometry(circle.InitialPose());
        // drivetrain.ResetOdometry(backFromWall.InitialPose());
        // drivetrain.ResetOdometry(BluePosition2.InitialPose());
        // drivetrain.ResetOdometry(BluePosition3.InitialPose());
        // drivetrain.ResetOdometry(turn180.InitialPose());





      // frc2::SequentialCommandGroup* trajectoryOneGroup = new frc2::SequentialCommandGroup(
      //   std::move(ramseteCommandTrajectoryOne),
      //   frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      //   // Auto Aim
      //   //Shooter
      // );
      frc2::SequentialCommandGroup* startGameGroup = new frc2::SequentialCommandGroup(
        std::move(ramseteCommandStartGame),
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      );
      // frc2::SequentialCommandGroup* circleGroup = new frc2::SequentialCommandGroup(
      //   std::move(ramseteCommandCircle),
      //   frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      // );
      
      // frc2::SequentialCommandGroup* bluePosition3Group = new frc2::SequentialCommandGroup(
        
      //   frc2::ParallelRaceGroup(
      //     std::move(ramseteCommandBluePosition3),
      //     frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake})
      //   ),

      //   frc2::ParallelRaceGroup(
      //     frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret})

      //   ),

      //   frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      // );

      // frc2::SequentialCommandGroup* backFromWallGroup = new frc2::SequentialCommandGroup(
      //   std::move(ramseteCommandBackFromWall),
      //   frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      // );
      //       frc2::SequentialCommandGroup* rotate180Group = new frc2::SequentialCommandGroup(
      //   std::move(ramseteCommandRotate180),
      //   frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {})
      // );



      // frc2::SequentialCommandGroup* bluePosition2Group = new frc2::SequentialCommandGroup(
        
      //   frc2::ParallelRaceGroup(
      //     std::move(ramseteCommandBluePosition2),
      //     frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake})
      //     ),

      //   std::move(ramseteCommandRotate180),

      //   frc2::ParallelCommandGroup(
      //     frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),  // ADD find a way to stop shooting
      //     frc2::InstantCommand([this] {m_shooter.setShooter();}, {&m_shooter})
      //   ),

      //   std::move(ramseteCommandRotate180),
        
      //   frc2::ParallelRaceGroup(
      //     std::move(ramseteCommandBluePosition2_1),
      //     frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake}) // ADD stop the instake
      //   ),

      //   std::move(ramseteCommandBackFromWall),
      //   std::move(ramseteCommandRotate180),
      //   std::move(ramseteCommandBluePosition2_2),
        
      //   frc2::ParallelCommandGroup(
      //     frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),  // ADD find a way to stop shooting
      //     frc2::InstantCommand([this] {m_shooter.setShooter();}, {&m_shooter})
      //   )


      // );

      // frc2::SequentialCommandGroup* bluePosition1Group = new frc2::SequentialCommandGroup(
      
      //   frc2::ParallelRaceGroup(
      //   std::move(ramseteCommandBluePosition1),
      //   frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake})
      // ),
      
      // std::move(ramseteCommandBackFromWall),
      // std::move(ramseteCommandRotate180),

      // frc2::ParallelCommandGroup(
      //   frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),  // ADD find a way to stop shooting
      //   frc2::InstantCommand([this] {m_shooter.setShooter();}, {&m_shooter})
      // ) 


      // );

      // THERE CANNOT BE INSTANT COMANDS IN PARALLEL RACE GROUPS
      frc2::SequentialCommandGroup* pickUpCloseBallGroup = new frc2::SequentialCommandGroup(
        Auto(drivetrain, 0.3, 0.5),
        Auto(drivetrain, 0.5, -0.2),
        frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake}),
        frc2::InstantCommand([this] {m_shooter.setShooter();}, {&m_shooter}),
        TimerCMD(3),
        frc2::InstantCommand([this] {drivetrain.ResetOdometry(turn180.InitialPose());}),
        std::move(ramseteCommandTurn180),
          frc2::ParallelCommandGroup(
          frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),
          TimerCMD(2)
        ),
        frc2::InstantCommand([this] {m_turret.setManuelAimOn();}, {&m_turret}),
        frc2::InstantCommand([this] {intake.IntakeInRelease();}, {&intake}),
        frc2::ParallelCommandGroup(
          TimerCMD(0.5),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        TimerCMD(0.5),
        
        frc2::ParallelCommandGroup(
          TimerCMD(0.5),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        frc2::InstantCommand([this] {m_shooter.stopShooter();}, {&m_shooter}),
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {&drivetrain})

        
      );
    

      frc2::SequentialCommandGroup* runShooterTestGroup = new frc2::SequentialCommandGroup (
        SafeBallShoot(intake, m_shooter, m_turret, 15)
      );
 


  // An example command will be run in autonomous
      // return trajectoryOneGroup;
      // return startGameGroup;
      // return circleGroup;
      // return bluePosition3Group;
      // return backFromWallGroup;
      // return rotate180Group;
      // return pickUpCloseBallGroup;
      return runShooterTestGroup;


  // return &m_autonomousCommand;

//  return Auto(drivetrain, 1.0, -4.0);
}


frc2::Command* RobotContainer::GetThreeBallAuto(){
  std::string threeBall1_1File = frc::filesystem::GetDeployDirectory() + "/paths/3Ball1.1.wpilib.json";
    frc::Trajectory threeBall1_1 = frc::TrajectoryUtil::FromPathweaverJson(threeBall1_1File);

  std::string threeBall1_2File = frc::filesystem::GetDeployDirectory() + "/paths/3Ball1.2.wpilib.json";
    frc::Trajectory threeBall1_2 = frc::TrajectoryUtil::FromPathweaverJson(threeBall1_2File);

  std::string threeBall1_3File = frc::filesystem::GetDeployDirectory() + "/paths/3Ball1.3.wpilib.json";
    frc::Trajectory threeBall1_3 = frc::TrajectoryUtil::FromPathweaverJson(threeBall1_3File);

  std::string threeBall1_4File = frc::filesystem::GetDeployDirectory() + "/paths/3Ball1.4.wpilib.json";
    frc::Trajectory threeBall1_4 = frc::TrajectoryUtil::FromPathweaverJson(threeBall1_4File);

  drivetrain.ResetOdometry(threeBall1_1.InitialPose());


  frc2::RamseteCommand ramseteCommandThreeBall1_1(
      threeBall1_1, [this]() { return drivetrain.GetPose(); },
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

frc2::RamseteCommand ramseteCommandThreeBall1_2(
      threeBall1_2, [this]() { return drivetrain.GetPose(); },
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

frc2::RamseteCommand ramseteCommandThreeBall1_3(
      threeBall1_3, [this]() { return drivetrain.GetPose(); },
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

frc2::RamseteCommand ramseteCommandThreeBall1_4(
      threeBall1_4, [this]() { return drivetrain.GetPose(); },
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



      frc2::SequentialCommandGroup* pickUp3BallsGroup = new frc2::SequentialCommandGroup(
        // Intake out and move
        Auto(drivetrain, 0.3, 0.5),
        Auto(drivetrain, 0.5, -.2),
        frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake}),
        frc2::InstantCommand([this] {m_shooter.setShooter();}, {&m_shooter}),
        std::move(ramseteCommandThreeBall1_1),
        std::move(ramseteCommandThreeBall1_2),
        
        // Auto Aim
        frc2::ParallelCommandGroup(
          frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),
          TimerCMD(2)
        ),
        frc2::InstantCommand([this] {m_turret.setManuelAimOn();}, {&m_turret}),

        // Shoot Twice
        frc2::ParallelCommandGroup(
          TimerCMD(0.5),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        TimerCMD(0.5),
        frc2::ParallelCommandGroup(
          TimerCMD(0.5),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),


        std::move(ramseteCommandThreeBall1_3),
        std::move(ramseteCommandThreeBall1_4),
        // Auto Aim Again
        frc2::ParallelCommandGroup(
          frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),
          TimerCMD(2)
        ),
        frc2::InstantCommand([this] {m_turret.setManuelAimOn();}, {&m_turret}),

        // Shoot Twice Again
        frc2::ParallelCommandGroup(
          TimerCMD(0.5),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        TimerCMD(0.5),
        frc2::ParallelCommandGroup(
          TimerCMD(0.5),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        frc2::InstantCommand([this] {m_shooter.stopShooter();}, {&m_shooter}),
        frc2::InstantCommand([this] {intake.IntakeInRelease();}, {&intake}),
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {&drivetrain})



      );
      return pickUp3BallsGroup;
      
      }
