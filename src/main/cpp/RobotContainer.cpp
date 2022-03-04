// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() : m_autonomousCommand(drivetrain,2.0,-4.0){
  // Initialize all of your commands and subsystems here
  drivetrain.SetDefaultCommand(TankDrive(drivetrain, stick1, stick2));
  m_turret.SetDefaultCommand(TurretCmd(m_turret, stick1, stick2, xbox));
  // Configure the button bm_indings
  ConfigureButtonBindings();

  cameraFishEye = frc::CameraServer::StartAutomaticCapture();

  cameraSelection = nt::NetworkTableInstance::GetDefault().GetTable("")->GetEntry("cameraSeleciton");
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
  frc2::JoystickButton(&stick2, 1).WhenPressed(&m_intakeIn); //wwas xbox 4
  frc2::JoystickButton(&stick2, 1).WhenReleased(&m_intakeInRelease); 

  frc2::JoystickButton(&stick1, 1).WhenPressed(&m_intakeOut); //was xbox 2
  frc2::JoystickButton(&stick1, 1).WhenReleased(&m_intakeOutRelease); 

  frc2::JoystickButton(&xbox, 3).WhenPressed(&m_conveyerForward); //was xbox 1
  frc2::JoystickButton(&xbox, 3).WhenReleased(&m_conveyerForwardRelease); //was xbox 1

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
  // An example command will be run in autonomous
  return &m_autonomousCommand;
//  return Auto(drivetrain, 1.0, -4.0);
}

void RobotContainer::updateCameras() {

  if(stick2.GetRawButton(15) == true && lastButtonState == false){
    
    if(cameraIsOn == false){
      // turn on camera
      // cameraSelection.SetString(cameraFishEye.GetName());

      // server.SetSource(cameraFishEye);

      frc::CameraServer::AddServer(cameraFishEye.GetName());

      cameraIsOn = true;

    } else if(cameraIsOn == true){
      // turn off camera
      // cameraSelection.SetString(NULL);

     frc::CameraServer::RemoveServer(cameraFishEye.GetName());

     cameraIsOn = false;
    }

    lastButtonState = true;

  } else if(stick2.GetRawButton(15) == false && lastButtonState == true){
    
    lastButtonState = false;
  }

  frc::SmartDashboard::PutBoolean("camera is on?", cameraIsOn);
}