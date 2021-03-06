// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>


void Robot::RobotInit() {
  m_chooser.AddOption("None", nullptr);
  m_chooser.SetDefaultOption("Close Ball Auto", m_container.GetCloseBallAuto());
  m_chooser.AddOption("Three Ball Auto", m_container.GetThreeBallAuto());
  m_chooser.AddOption("Two Ball Auto", m_container.GetTwoBallAuto());
  m_chooser.AddOption("One And A Half Auto", m_container.GetOneBallPlusOneAuto());
  m_chooser.AddOption("DriveBack_Shoot", m_container.GetDriveBack_Shoot());
  frc::SmartDashboard::PutData(&m_chooser);

  // automatically start camera server
  frc::CameraServer::StartAutomaticCapture();
}
/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  // std::ofstream log_file; 
  // log_file.open("logfile.txt");
  // log_file << m_container.GetLog();
  // log_file.close();
  // frc::SmartDashboard::PutData(frc2::CommandScheduler::GetInstance().Run());
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_chooser.GetSelected();

  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != nullptr) {
      m_autonomousCommand->Cancel();
      // delete m_autonomousCommand;
      // m_autonomousCommand = nullptr;
    }
}


/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
