#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <units/length.h>
#include <units/voltage.h>
//#include <wpi/math>

#pragma once 

namespace DriveConstants{ 

    constexpr auto ks = 0.18757_V; //0.162631_V
    constexpr auto kv = 1.3814 * 1_V * 1_s / 1_m; //1.392
    constexpr auto ka = 0.25812 * 1_V * 1_s * 1_s / 1_m; //0.27535
    // constexpr double kPDriveVel = 1.69; 
    constexpr double kPDriveVel = 0.00001; 
    constexpr auto trackWidth = 0.58_m; 
    //constexpr auto trackWidth = 0.784_m; //February 2022
    extern const frc::DifferentialDriveKinematics kDriveKinematics;
    constexpr int kEncoderCPR = 1024; 

    // constexpr auto ks = 0.2_V; // 0.131_V; 
    // constexpr auto kv = 2.84 * 1_V * 1_s / 1_m;     //  2.73 * 1_V * 1_s / 1_m; 
    // constexpr auto ka = 0.373 * 1_V * 1_s * 1_s / 1_m;    //.516 * 1_V * 1_s * 1_s / 1_m; 
    // // constexpr double kPDriveVel = 1.69; 
    // constexpr double kPDriveVel = 0.00001; 
    // constexpr auto trackWidth = 0.58_m;       //0.589_m; 
    // extern const frc::DifferentialDriveKinematics kDriveKinematics;
    // constexpr int kEncoderCPR = 1024; 


}

namespace AutoConstants{ 
     constexpr auto kMaxSpeed = 2.2_mps; 
     constexpr auto kMaxAcceleration = 1.2_mps_sq;
     constexpr double  kRamseteB = 2; 
     constexpr double kRamseteZeta = .7;
     
}

