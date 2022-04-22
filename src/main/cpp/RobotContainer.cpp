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

std::string RobotContainer::GetLog() {
  return m_turret.GetLog();
}
void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  frc2::JoystickButton(&stick2, 1).WhenPressed(&m_intakeIn); //wwas xbox 4
  frc2::JoystickButton(&stick2, 1).WhenReleased(&m_intakeInRelease); 

  frc2::JoystickButton(&stick1, 1).WhenPressed(&m_intakeOut); //was xbox 2
  frc2::JoystickButton(&stick1, 1).WhenReleased(&m_intakeOutRelease); 

  // configure to PreAngle command
  frc2::JoystickButton(&stick1, 5).WhenPressed(PreAngles(m_turret, 20.0)); // stick1||2 5-16 or xbox 8


  frc2::JoystickButton(&xbox, 3).WhenPressed(&m_conveyerForward); //was xbox 1
  frc2::JoystickButton(&xbox, 3).WhenReleased(&m_conveyerForwardRelease); 

  frc2::JoystickButton(&xbox, 1).WhenPressed(&m_conveyerBackward); //was stick2 2
  frc2::JoystickButton(&xbox, 1).WhenReleased(&m_conveyerBackwardRelease); 

  // frc2::JoystickButton(&xbox, 3).WhenPressed(&m_shooter2000); 
  // frc2::JoystickButton(&xbox, 3).WhenReleased(&m_shooterStop); 

  frc2::JoystickButton(&stick2, 11).WhenPressed(&m_shooter2000); 
  frc2::JoystickButton(&stick2, 12).WhenPressed(&m_shooterStop); 
  frc2::JoystickButton(&stick1, 11).WhenPressed(&m_shooter2000); 
  frc2::JoystickButton(&stick1, 12).WhenPressed(&m_shooterStop); 

  frc2::JoystickButton(&xbox, 4).WhenPressed(&m_shooterReverse);
  
  frc2::JoystickButton(&xbox, 7).WhenPressed(&m_TurtModeAuto); 
  frc2::JoystickButton(&xbox, 7).WhenReleased(&m_TurtModeManu); 

  frc2::JoystickButton(&stick2, 16).WhenPressed(&m_verticalAim);
  //frc2::JoystickButton(&stick2, 15).WhenPressed(&m_lowGoalAim);

  frc2::JoystickButton(&xbox, 6).WhenPressed(&m_ClimbServoLock); //right bumper
  frc2::JoystickButton(&xbox, 5).WhenPressed(&m_ClimbServoUnlock); //left bumper

  
}

frc2::Command* RobotContainer::GetTwoBallAuto() {
  //frc::TrajectoryConfig config{AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}; 
  //config.SetKinematics(DriveConstants::kDriveKinematics);

  std::string twoBall1_1File = frc::filesystem::GetDeployDirectory() + "/paths/output/2Ball1.1.wpilib.json";
  twoBall1_1 = frc::TrajectoryUtil::FromPathweaverJson(twoBall1_1File);

  std::string twoBall1_2File = frc::filesystem::GetDeployDirectory() + "/paths/output/2Ball1.2.wpilib.json";
  twoBall1_2 = frc::TrajectoryUtil::FromPathweaverJson(twoBall1_2File);

  std::string twoBall1_3File = frc::filesystem::GetDeployDirectory() + "/paths/output/2Ball1.3.wpilib.json";
  twoBall1_3 = frc::TrajectoryUtil::FromPathweaverJson(twoBall1_3File);

  frc2::RamseteCommand ramseteCommandTwoBall1_1(
      twoBall1_1, [this]() { return drivetrain.GetPose(); },
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

      frc2::RamseteCommand ramseteCommandTwoBall1_2(
      twoBall1_2, [this]() { return drivetrain.GetPose(); },
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

      frc2::RamseteCommand ramseteCommandTwoBall1_3(
      twoBall1_3, [this]() { return drivetrain.GetPose(); },
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

      frc2::SequentialCommandGroup* twoBallGroup = new frc2::SequentialCommandGroup(
       
        //shake intake down
        //frc2::InstantCommand([this] {drivetrain.ResetOdometry(twoBall1_1.InitialPose());}),
        frc2::InstantCommand([this] {frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 3050); 
          frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 3050);}),
        frc2::InstantCommand([this] {drivetrain.ResetOdometry(twoBall1_1.InitialPose());}),

        frc2::InstantCommand([this] {m_shooter.setShooter(3050,3050);}, {&m_shooter}),
        Auto(drivetrain, 0.3, 0.5),
        Auto(drivetrain, 0.5, -0.2),
        //Turn on intake and spin up shooter

        frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake}),
       
      
        std::move(ramseteCommandTwoBall1_1),
        std::move(ramseteCommandTwoBall1_2),
        /*   frc2::ParallelCommandGroup(
           frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),
           TimerCMD(1)
         ),*/
        SafeBallShoot(m_turret, 20), //  TODO FIX 2 BALL AUTO
        frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),

        //frc2::InstantCommand([this] {m_turret.setManualAimOn();}, {&m_turret}),
        //frc2::InstantCommand([this] {intake.IntakeInRelease();}, {&intake}),
        frc2::ParallelCommandGroup(
          TimerCMD(0.3),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        TimerCMD(0.5),
        
        frc2::ParallelCommandGroup(
          TimerCMD(0.3),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        //TimerCMD(0.5),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        frc2::InstantCommand([this] {m_turret.setManualAimOn();}, {&m_turret}),
        std::move(ramseteCommandTwoBall1_3),
       /* frc2::ParallelCommandGroup(
          frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),
          TimerCMD(1)
        ),
         frc2::InstantCommand([this] {m_turret.setManualAimOn();}, {&m_turret}),
        */ 
        SafeBallShoot(m_turret,20),
        frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),
        
        frc2::ParallelCommandGroup(
          TimerCMD(5),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        
        frc2::InstantCommand([this] {intake.IntakeInRelease();},{&intake}),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {&drivetrain}),

        frc2::InstantCommand([this] {m_turret.setManualAimOn();}, {&m_turret})

      );
      //std::cout<<twoBall1_1File<<std::endl;
      return twoBallGroup;
}
frc2::Command* RobotContainer::GetCloseBallAuto() {

  frc::TrajectoryConfig config{AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}; 
  config.SetKinematics(DriveConstants::kDriveKinematics);
  

  std::string turn180File = frc::filesystem::GetDeployDirectory() + "/paths/output/Turn180_wider.wpilib.json";
  turn180 = frc::TrajectoryUtil::FromPathweaverJson(turn180File);




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

     
      
      // THERE CANNOT BE INSTANT COMANDS IN PARALLEL RACE GROUPS
      frc2::SequentialCommandGroup* pickUpCloseBallGroup = new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] {frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 3050); 
          frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 3050);}),
        frc2::InstantCommand([this] {drivetrain.ResetOdometry(turn180.InitialPose());}),
        frc2::InstantCommand([this] {m_shooter.setShooter(3050,3050);}, {&m_shooter}),

        Auto(drivetrain, 0.3, 0.5),
        Auto(drivetrain, 0.5, -0.2),
        frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake}),
        std::move(ramseteCommandTurn180),
        //   frc2::ParallelCommandGroup(
        //   frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),
        //   TimerCMD(2)
        // ),
        // frc2::InstantCommand([this] {m_turret.setManualAimOn();}, {&m_turret}),
        SafeBallShoot(m_turret, 20),

        frc2::InstantCommand([this] {intake.IntakeInRelease();}, {&intake}),
        frc2::ParallelCommandGroup(
          TimerCMD(0.3),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        TimerCMD(1.0),
        
        frc2::ParallelCommandGroup(
          TimerCMD(1.0),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {&drivetrain})

        
      );
    
      return pickUpCloseBallGroup;
      // return runShooterTestGroup;


  // return &m_autonomousCommand;

//  return Auto(drivetrain, 1.0, -4.0);
}







//Lime Light Lock

frc2::Command* RobotContainer::GetLimelightLockOn(){
  frc2::SequentialCommandGroup* lockOnGroup = new frc2::SequentialCommandGroup{
    SafeBallShoot(m_turret, 20), 
    frc2::ParallelCommandGroup(
      TimerCMD(0.5),
      frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
    ),
    frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),

  };
  return lockOnGroup;
}

//DriveBack_Shoot
frc2::Command* RobotContainer::GetDriveBack_Shoot() {

  frc::TrajectoryConfig config{AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}; 
  config.SetKinematics(DriveConstants::kDriveKinematics);
  

  std::string DriveBack_ShootFile = frc::filesystem::GetDeployDirectory() + "/paths/output/DriveBack_Shoot.wpilib.json";
  DriveBack_Shoot = frc::TrajectoryUtil::FromPathweaverJson(DriveBack_ShootFile);




frc2::RamseteCommand ramseteCommandDriveBack(
      DriveBack_Shoot, [this]() { return drivetrain.GetPose(); },
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

     
      
      // THERE CANNOT BE INSTANT COMANDS IN PARALLEL RACE GROUPS
      frc2::SequentialCommandGroup* DriveBack_ShootGroup = new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] {frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 3050); 
          frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 3050);}),
        frc2::InstantCommand([this] {drivetrain.ResetOdometry(DriveBack_Shoot.InitialPose());}),
        frc2::InstantCommand([this] {m_shooter.setShooter(3050,3050);}, {&m_shooter}),

        Auto(drivetrain, 0.3, 0.5),
        Auto(drivetrain, 0.5, -0.2),
        //frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake}),
        std::move(ramseteCommandDriveBack),
        //   frc2::ParallelCommandGroup(
        //   frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),
        //   TimerCMD(2)
        // ),
        // frc2::InstantCommand([this] {m_turret.setManualAimOn();}, {&m_turret}),
        SafeBallShoot(m_turret, 20),
        frc2::InstantCommand([this] {m_turret.setAutoAimOn();}, {&m_turret}),
         
        
        frc2::InstantCommand([this] {intake.IntakeInRelease();}, {&intake}),
        frc2::ParallelCommandGroup(
          TimerCMD(0.3),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        TimerCMD(1.0),
         
        /*frc2::ParallelCommandGroup(
          TimerCMD(1.0),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),*/
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {&drivetrain}),

        frc2::InstantCommand([this] {m_turret.setManualAimOn();}, {&m_turret})

      );
    
      return DriveBack_ShootGroup;
      // return runShooterTestGroup;


  // return &m_autonomousCommand;

//  return Auto(drivetrain, 1.0, -4.0);
}
//One and a half auto

frc2::Command* RobotContainer::GetOneBallPlusOneAuto(){
  std::string OneBallPlusOne1_1File = frc::filesystem::GetDeployDirectory() + "/paths/output/OneBallPlusOne1.1.wpilib.json";
    frc::Trajectory OneBallBallPlusOne1_1 = frc::TrajectoryUtil::FromPathweaverJson(OneBallPlusOne1_1File);

  std::string OneBallPlusOne1_2File = frc::filesystem::GetDeployDirectory() + "/paths/output/OneBallPlusOne1.2.wpilib.json";
    frc::Trajectory OneBallPlusOne1_2 = frc::TrajectoryUtil::FromPathweaverJson(OneBallPlusOne1_2File);


 frc2::RamseteCommand ramseteCommandOneBallPlusOne1_1(
      OneBallPlusOne1_1, [this]() { return drivetrain.GetPose(); },
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


      frc2::RamseteCommand ramseteCommandOneBallPlusOne1_2(
      OneBallPlusOne1_2, [this]() { return drivetrain.GetPose(); },
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

       frc2::SequentialCommandGroup* pickUpCloseBallGroupPlusOne = new frc2::SequentialCommandGroup(
        frc2::InstantCommand([this] {frc::SmartDashboard::PutNumber("Shooter Set RPM 2 F", 3000); 
          frc::SmartDashboard::PutNumber("Shooter Set RPM 2 B", 3000);}),
        frc2::InstantCommand([this] {drivetrain.ResetOdometry(turn180.InitialPose());}),

        Auto(drivetrain, 0.3, 0.5),
        Auto(drivetrain, 0.5, -0.2),
        frc2::InstantCommand([this] {intake.IntakeIn();}, {&intake}),
        frc2::InstantCommand([this] {m_shooter.setShooter();}, {&m_shooter}),
        std::move(ramseteCommandOneBallPlusOne1_1),
 
        SafeBallShoot(m_turret, 20),

        frc2::InstantCommand([this] {intake.IntakeInRelease();}, {&intake}),
        frc2::ParallelCommandGroup(
          TimerCMD(0.3),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
        TimerCMD(1.0),
        
        frc2::ParallelCommandGroup(
          TimerCMD(1.0),
          frc2::InstantCommand([this] {intake.ConveyorForward();}, {&intake})
        ),
        frc2::InstantCommand([this] {intake.ConveyorForwardRelease();}, {&intake}),
         std::move(ramseteCommandOneBallPlusOne1_2),

        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {&drivetrain})

        
      );
      return pickUpCloseBallGroupPlusOne;
      }


//There Ball Auto

      frc2::Command* RobotContainer::GetThreeBallAuto(){
  std::string threeBall1_1File = frc::filesystem::GetDeployDirectory() + "/paths/output/3Ball1.1.wpilib.json";
    frc::Trajectory threeBall1_1 = frc::TrajectoryUtil::FromPathweaverJson(threeBall1_1File);

  std::string threeBall1_2File = frc::filesystem::GetDeployDirectory() + "/paths/output/3Ball1.2.wpilib.json";
    frc::Trajectory threeBall1_2 = frc::TrajectoryUtil::FromPathweaverJson(threeBall1_2File);

  std::string threeBall1_3File = frc::filesystem::GetDeployDirectory() + "/paths/output/3Ball1.3.wpilib.json";
    frc::Trajectory threeBall1_3 = frc::TrajectoryUtil::FromPathweaverJson(threeBall1_3File);

  std::string threeBall1_4File = frc::filesystem::GetDeployDirectory() + "/paths/output/3Ball1.4.wpilib.json";
    frc::Trajectory threeBall1_4 = frc::TrajectoryUtil::FromPathweaverJson(threeBall1_4File);
// TODO MOVE THIS to after intake shake
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
        SafeBallShoot(m_turret, 20),
       
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
        SafeBallShoot(m_turret, 20),

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
        // frc2::InstantCommand([this] {m_shooter.stopShooter();}, {&m_shooter}),
        frc2::InstantCommand([this] {intake.IntakeInRelease();}, {&intake}),
        frc2::InstantCommand([this] { drivetrain.TankDriveVolts(0_V, 0_V); }, {&drivetrain})



      );
      //std::cout<< threeBall1_1File<<std::endl;
      return pickUp3BallsGroup;
      
      }