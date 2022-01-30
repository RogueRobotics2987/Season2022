// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <iostream>  // Only need if you use cout (don't)
//#include <units/time.h>   // all "units" libraries automatically include all fmt comamnds
                            // currently all robotics (timed and command-based) include units automatically.

//  You may need one of the below items if you are running CLI C++, it is good practice to include one.
//#include <fmt/format.h> // Technically the required library for fmt::format
//#include <fmt/printf.h> // Technically the required library for fmt::print
//#indclude <fmt/core.h>  // Techinically the required library for all fmt functions.

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;
};
