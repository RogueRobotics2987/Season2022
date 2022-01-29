// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterActuator.h"

ShooterActuator::ShooterActuator(){
    angleMotorH.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);// was brake mode
    angleMotorV.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);//was brake mode
    frc::SmartDashboard::PutNumber("Shooter Aim - Horz. - P", AimH_P);
    myTimer = frc::Timer();
    myTimer.Reset();
    myTimer.Start();


}

double ShooterActuator::GetTX(){
    return tx; 
}
double ShooterActuator::GetTY(){
    return ty; 
}
void ShooterActuator::switchCam(bool flag){
    if(flag){
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("stream", 1); 
    }
    else{
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("stream", 2); 
    }
}

void ShooterActuator:: limeStream(int num){
   nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr") -> PutNumber("pipeline", num);
}
// This method will be called once per scheduler run
void ShooterActuator::Periodic() {
       units::time::second_t startTime = myTimer.Get();

    bool shooterActuatorWorking = true; 
    if(angleMotorH.GetFirmwareString() != firmwareVersion || angleMotorV.GetFirmwareString() != firmwareVersion){
         shooterActuatorWorking = false; 
    }
    
    frc::SmartDashboard::PutBoolean("Shooter Actuator Working", shooterActuatorWorking); 

    tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->GetNumber("tx", 0.0); 
    ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")-> GetNumber("ty", 0.0); 


    // Control Loop Vals
    AimH_P = frc::SmartDashboard::GetNumber("Shooter Aim - Horz. - P", 0.0);

    frc::SmartDashboard::PutNumber("Limelight X", tx);
    frc::SmartDashboard::PutNumber("Limelight Y", ty);

    //Mode 1: Startup
    //  Set Motor speed to inwards
    //  Until LimitSwitch triggers
    bool LimitSwitchStateF = angleMotorH.GetForwardLimitSwitch(
        rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    frc::SmartDashboard::PutBoolean("F Limit Switch State", LimitSwitchStateF);
    
    PositionH = angleMotorH.GetEncoder().GetPosition();
    frc::SmartDashboard::PutNumber("Shooter Position - Horz.", PositionH);
    PositionV = angleMotorV.GetEncoder().GetPosition();
    frc::SmartDashboard::PutNumber("Shooter Position - Vert.", PositionV);

    bool SoftLimitSwitchR = angleMotorH.IsSoftLimitEnabled(rev::CANSparkMax::SoftLimitDirection::kReverse);
    bool SoftLimitSwitchF = angleMotorH.IsSoftLimitEnabled(rev::CANSparkMax::SoftLimitDirection::kForward);
    frc::SmartDashboard::PutBoolean("softF Limit Switch State", SoftLimitSwitchF);
    frc::SmartDashboard::PutBoolean("softR Limit Switch State", SoftLimitSwitchR);
   
    units::time::second_t EndTime = myTimer.Get();
    units::time::second_t RunTime= EndTime - startTime;
    if(0) {
       std::cout << "ShooterActuator Periodic Time: " << RunTime.value()<< std::endl;
    }


}
bool ShooterActuator::GetForwardLimitState() {
    bool LimitSwitchStateF = angleMotorH.GetForwardLimitSwitch(
    rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    return LimitSwitchStateF;
}

void ShooterActuator::SetAutoAim(bool AutoAimFlag) {
    AutoAimMode = AutoAimFlag;
}

void ShooterActuator::setAngleH(double stickVal){
    frc::SmartDashboard::PutNumber("Aim State - Horz.", H_AimState);
    bool LimitSwitchStateF = angleMotorH.GetForwardLimitSwitch(
    rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed).Get();
    //float PositionH = angleMotorH->GetEncoder().GetPosition();

    if(H_AimState == 0) {  //Startup State
        safeSetH(1.0);
        if(LimitSwitchStateF) {
            H_AimState = 9;
            angleMotorH.GetEncoder().SetPosition(0);
        }
    } else if (H_AimState == 3) { // Auto Aim Mode
        //ledMode 
        // 3 on 
        // 0 off 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("ledMode", 3); 

        float H_ControlVal = AimH_P*tx;
        frc::SmartDashboard::PutNumber("Temp Aiming Horz. Motor Output", H_ControlVal);
        safeSetH(H_ControlVal); // For now do nothing...
        //safeSetV(0.0); // For now do nothing...

        if(!AutoAimMode) {
            H_AimState = 9;
        }

        
    } else if (H_AimState == 9) { 
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-rr")->PutNumber("ledMode", 0); 

        // Manual Aim Mode
        // stickVal = safeStick(stickVal, PositionH);
        // angleMotorH->Set(stickVal);
        safeSetH(stickVal); // For now do nothing...


        if(AutoAimMode) {
            H_AimState = 3;
        }

    } else {  // We should NEVER get here!
      std::cout << "WARNING SHOOT IN BAD STATE: " << H_AimState << std::endl;
    }

}
double ShooterActuator::safeStick(double stickVal, double pos) {
    double curMax = 0;
    double curMin =0;
    if(fabs(stickVal) < .18) { stickVal = 0.0;} //stickVal was <.08 but the deadzone was off
    // if(stickVal>0.4) { // Limit to 0.4 max power
    //     stickVal = 0.4;
    // } else if(stickVal<-0.4) {
    //     stickVal = -0.4;
    // } 

    if(pos < -100){
        curMax = 0.9;
    } else if (-100 < pos && pos <= 0) {
        curMax = (1.0 - 0.4)/(-100.0 -0.0)* pos + 0.4; 
    } else {
        curMax = 0.4;
    }

        
    if(-400 < pos){
        curMin = -0.9;
    } else if (-500 <= pos && pos <= -400) {
        curMin = (-0.4 - - 1.0)/(-500 - -400)* pos + -3.4; 
    } else {
        curMin = -0.4;
    }

    stickVal = std::min(curMax,stickVal);
    stickVal = std::max(curMin,stickVal);

    return stickVal; 

}
    void ShooterActuator::safeSetH(double setVal){
        //double aimResetSpeedH = 1.0;

        // stickVal = safeStick(stickVal, PositionH);
        // angleMotorH->Set(stickVal);

        setVal = safeStick(setVal, PositionH);
        angleMotorH.Set(setVal);

    }

    void ShooterActuator::safeSetV(double setVal){
        //double aimResetSpeedV = 1.0;
        setVal = safeStick(setVal, PositionV);
        angleMotorV.Set(setVal);

    }


