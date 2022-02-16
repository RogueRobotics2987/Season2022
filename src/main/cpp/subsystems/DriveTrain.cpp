// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"
#include <iostream>
#include <cmath>

DriveTrain::DriveTrain() {

    SetName("DriveTrain");
    LeftBack.Follow(LeftFront);
    RightBack.Follow(RightFront);
    LeftFront.SetInverted(true);
    RightFront.SetInverted(false); 
    DriveTrain::Reset();


    frc::SmartDashboard::PutNumber("Set P", Lvkp); 
    frc::SmartDashboard::PutNumber("Set I", Lvki); 
    frc::SmartDashboard::PutNumber("Set D", Lvkd);
    
    frc::SmartDashboard::PutNumber(" P", Avkp); 
    frc::SmartDashboard::PutNumber(" I", Avki); 
    frc::SmartDashboard::PutNumber(" D", Avkd);

    frc::SmartDashboard::PutNumber("CurrentDistance",currentPitch );
    frc::SmartDashboard::PutNumber("CurrentAngle",currentHeading );

  //  DrivePID = new rev::CANPIDController(shooterMotor);

}

void DriveTrain::Drive(double xSpeed, double zRotation) {
    m_robotDrive.ArcadeDrive(xSpeed, -zRotation);
}

void DriveTrain::autonDrive(){
    m_robotDrive.ArcadeDrive(-.4, 0); 
}

// This method will be called once per scheduler run
void DriveTrain::Periodic() {

    frc::SmartDashboard::PutNumber("Get Heading (ahrs)", myAhrs.GetAngle());
    frc::SmartDashboard::PutNumber("Get Heading (converted)", double(GetHeading()));
  
    frc::SmartDashboard::PutNumber("Output Voltage Right BusVolatage", RightFront.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Output Voltage Left BusVoltage", LeftFront.GetBusVoltage());
    frc::SmartDashboard::PutNumber("Output Voltage Right GetApplied", RightFront.GetAppliedOutput());
    frc::SmartDashboard::PutNumber("Output Voltage Left GetApplied", LeftFront.GetAppliedOutput());


    m_odometry.Update(
        frc::Rotation2d(GetHeading()), 
        units::meter_t(LeftEncoder.GetPosition() * 0.044), 
        units::meter_t(-1.0 * RightEncoder.GetPosition() * 0.044)
    );
  
    frc::SmartDashboard::PutNumber("left Encoder Val", LeftEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("right Encoder Val", -1.0 * RightEncoder.GetPosition());

  m_field.SetRobotPose(m_odometry.GetPose());
  frc::SmartDashboard::PutData("Field", &m_field);
}

void DriveTrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
//   // if(left > units::volt_t(.25)){ left = units::volt_t(.25); }
//   // if(right > units::volt_t(.25)){ right = units::volt_t(.25); }
  frc::SmartDashboard::PutNumber("Left Distance", LeftEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Right Distance", RightEncoder.GetPosition());
  frc::SmartDashboard::PutNumber("Tank Drive Volts Left", double(left));
  frc::SmartDashboard::PutNumber("Tank Drive Volts Right", double(right));
  // frc::SmartDashboard::PutNumber("AHRS Heading", GetHeading()); 
  
  LeftFront.SetVoltage(left);
  RightFront.SetVoltage(-right);
  m_robotDrive.Feed();

}
units::degree_t DriveTrain::GetHeading() { 
  return units::degree_t(-1.0 * myAhrs.GetAngle()); // TODO: Fixed Units
}


void DriveTrain::Reset() {
  myAhrs.Reset();
  LeftEncoder.SetPosition(0.0);
  RightEncoder.SetPosition(0.0);
} 

frc::DifferentialDriveWheelSpeeds DriveTrain::GetWheelSpeeds() {
  // units::meter_t(LeftEncoder.GetPosition() * 0.044), 
  // units::meter_t(-1.0 * RightEncoder.GetPosition() * 0.044)
  return {(LeftEncoder.GetVelocity() * 1_mps * 0.044 / 60),
      (-RightEncoder.GetVelocity() * 1_mps * 0.044 / 60)};
}

void DriveTrain::ResetOdometry(frc::Pose2d pose){ 
  Reset(); //reset encoders and ahrs  
  m_odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading()))); 
}

frc::Pose2d DriveTrain::GetPose(){
  return m_odometry.GetPose();
}

void DriveTrain::ResetEncoders(){ 
  RightEncoder.SetPosition(0); 
  LeftEncoder.SetPosition(0); 
}

double DriveTrain::CalculatePhi(double current_y){
  double phi;
  phi = fov_y*current_y/max_y;
  return phi;
}

double DriveTrain::CalculateTheta(double current_x){
  double theta;
  theta = current_x*fov_x/max_x;
  return theta;
}

void DriveTrain::JetsonControl(){
  
    std::vector< double > xArray = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetNumberArray("Left X", defaultValReturn);
    std::vector< double > yArray = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetNumberArray("Top Y",defaultValReturn);
    std::vector< std::string > labelArray = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetStringArray("Label",defaultStringReturn);
    std::vector< double > areaArray = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetNumberArray("Area",defaultValReturn);
    std::vector< double > confArray = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard")->GetNumberArray("Confidence",defaultValReturn);

  //  currentHeading = frc::SmartDashboard::GetNumber("CurrentHeading", currentHeading);
  //  currentPitch = frc::SmartDashboard::GetNumber("CurrentPitch", currentPitch);

  //filtering out large changes in x and y
  if(firstball_x.size()>0 && xArray.size() > 0){
    if(std::abs(calculateBallAverage(&firstball_x) - xArray[0]) > X_THRESH){
      xArray.erase(xArray.begin());
    }
  }

  frc::SmartDashboard::PutNumberArray("Filtered xArray", xArray);

  // if(firstball_y.size()>0 && yArray.size() > 0){
  //   if(std::abs(calculateBallAverage(&firstball_y) - yArray[0]) > Y_THRESH){
  //     yArray.erase(yArray.begin());
  //   }
  // }

  //calculating rolling average if necessary
  double x_arr_0 = 0;
  if(xArray.size()==0){
    std::cout<<"No ball detected"<< std::endl;
    if(firstball_x.size()==0){
      //nothing
    }
    else if(num_x_0 < max_undetected_iterations){
      std::cout<<"i am going to calculate ball average: "<< calculateBallAverage(&firstball_x)<< std::endl;
      x_arr_0 = calculateBallAverage(&firstball_x);
      firstball_x.push_back(x_arr_0); 
        if(firstball_x.size()>5){
          firstball_x.erase(firstball_x.begin());
        }
        std::cout<<"x_arr_0: "<<x_arr_0<<std::endl;
    }
    else if (num_x_0 >= max_undetected_iterations){
        firstball_x.clear();
    }
  }
  else {
    x_arr_0 = xArray[0];
    firstball_x.push_back(x_arr_0);
      if(firstball_x.size()>5){
        firstball_x.erase(firstball_x.begin());
   }
   std::cout<<"xArray 0: "<<xArray[0]<< std::endl;
  }

  if(x_arr_0==0){
    num_x_0 += 1;
    currentHeading = CalculateTheta(last_x_val);
  }
  else{
    last_x_val = x_arr_0;
    currentHeading = CalculateTheta(x_arr_0);
    num_x_0 = 0;
  }
    for(double val: firstball_x){
      std::cout<<"firstball_x: "<< val << std::endl;
    } 

  // if(xArray.size()==0 && firstball_x.size()>0){
  //   x_arr_0 = calculateBallAverage(&firstball_x);
  //   std::cout<<"x array 0: "<< x_arr_0<< std::endl;
    // for(double val: firstball_x){
    //   std::cout<<"firstball_x: "<< val << std::endl;
    // } 
  //   firstball_x.push_back(x_arr_0);
  //  }
  //  else if(xArray.size()>0){
  //    firstball_x.push_back(xArray[0]);
  //    x_arr_0 = xArray[0];
  //  }
  //  if(firstball_x.size()>5){
  //    firstball_x.erase(firstball_x.begin());
  //  }
   double y_arr_0 = 0; 
  // if(yArray.size()==0 && firstball_y.size()>0){
  //   y_arr_0 = calculateBallAverage(&firstball_y);
  //   firstball_y.push_back(y_arr_0);
  // }
  // else if(yArray.size()>0){
  //   firstball_y.push_back(yArray[0]);
  //   y_arr_0 = yArray[0];
  // }
  // if(firstball_y.size()>5){
  //    firstball_y.erase(firstball_y.begin());
  //  }
  //   std::cout<<"first ball x size: "<<firstball_x.size() << std::endl;
  //   std::cout<<"x array size: "<<xArray.size()<< std::endl;

    // currentHeading = CalculateTheta(x_arr_0);
    currentPitch = CalculatePhi(y_arr_0);

    frc::SmartDashboard::PutNumber("UpdatedCurrentAngle", currentHeading);
    frc::SmartDashboard::PutNumber("UpdatedCurrentDistance", currentPitch);

    frc::SmartDashboard::PutNumber("ballx", x_arr_0);
    frc::SmartDashboard::PutNumber("bally", y_arr_0);
    frc::SmartDashboard::PutNumberArray("ballxArray", xArray);


    double LvPidOut = LvPid.Calculate(currentPitch ,fov_y/2.0);
    double AvPidOut = AvPid.Calculate(currentHeading ,fov_x/2.0);
    std::cout<<AvPidOut<< std::endl;
    if (AvPidOut > 0.4){
      AvPidOut = 0.4;
    } else if (AvPidOut < -0.4){
      AvPidOut = -0.4;
    }

    if (LvPidOut > 0.25){
      LvPidOut = 0.25;
    } else if (LvPidOut < -0.25){
      LvPidOut = -0.25;
    }  


    frc::SmartDashboard::PutNumber("LvPid", LvPidOut);
    frc::SmartDashboard::PutNumber("AvPid", AvPidOut);

  // std::cout <<AvPidOut<< std::endl;

  if(num_x_0 > max_undetected_iterations){
  m_robotDrive.ArcadeDrive(0, 0); //turns in circle and searches 
  }
  else{
    m_robotDrive.ArcadeDrive(0, AvPidOut);
  }
}

double DriveTrain::calculateBallAverage(std::vector< double > * ball_Array){
double ballAverage = 0;
int arraySize = ball_Array->size();
// if(arraySize>=5){
//   arraySize = 4;
// }
// std::cout<<"array size: "<< arraySize << std::endl;
// std::cout<<"ball array size: "<<ball_Array->size()<<std::endl;
// for(int i = ball_Array->size()-1; i>=0; i--){
//   ballAverage += (*ball_Array)[i];
//   std::cout<<"adding value "<<(*ball_Array)[i]<<" to get us a total of "<< ballAverage<< std::endl;
// }
for(double val: *ball_Array){
  ballAverage += val;
  // std::cout<<"ball average while adding up: "<<ballAverage<<" plus val: "<<val<< std::endl;
}
//std::cout<<ballAverage<< std::endl;
ballAverage = (double)ballAverage/arraySize;

// ballAverage = (*ball_Array)[0] + (*ball_Array)[1] + (*ball_Array)[2] + (*ball_Array)[3] + (*ball_Array)[4];
// ballAverage = ballAverage/5;
// std::cout<<"Ball average: "<<ballAverage<< std::endl;
return ballAverage;
};