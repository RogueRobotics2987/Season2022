// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit() {}
void Robot::RobotPeriodic() {
  // Example stuff we might want to print
  double myDouble = 1.2; // Create a double 
  units::second_t myTime = units::second_t(1.2);  // Create a variable with "seconds" units
  // NOTE: "seconds" are the output type of the new Timer.Get() function.

  // Old way, needs <iostream> library
  std::cout << "My time is " << 1.2 << "seconds" << std::endl;
  std::cout << "My time is " << myDouble << "seconds" << std::endl;
  std::cout << "My time is " << myTime.value() << "seconds" << std::endl;
  //std::cout << myDouble.value(); // Doesn't Work! "value" only works on units

  // New way, called "fmt" commands 
  //   automatically included with all of the <units/*> libraries
  fmt::print("My time is {} seconds", 1.2);
  fmt::print("My time is {} seconds", myDouble);
  fmt::print("My time is {} seconds", myTime);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
