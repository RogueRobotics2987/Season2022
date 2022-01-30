// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <iostream>  // Only need if you use cout (don't)
#include <units/time.h> // Only include the units you actually need/use.
                        // all "units" libraries automatically include all fmt comamnds

#include <fmt/format.h> // Technically the required library for fmt::format
#include <fmt/printf.h> // Technically the required library for fmt::print

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
