// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() {
    frc::SmartDashboard::PutNumber("Conveyor Value", .3);

    //p_conveyorMotor = new rev::CANSparkMax(rev::CANSparkMax::MotorType::kBrushless, 51);
   // p_intakeSensor = frc::DigitalInput(1);
    myTimer = frc::Timer(); 
    //intakeSolenoid = frc::DoubleSolenoid(0,1); 
    p_intakeMotor.SetSmartCurrentLimit(0); 
   // p_topSensor = frc::DigitalInput(2);
    intakeTimer = frc::Timer(); 
    intakeTimer.Start();
    intakeTimer.Reset(); 
    // climbingPiston = new frc::DoubleSolenoid(6,7);//DoubleSolenoid(7); 
    myTimer2 = frc::Timer();
    myTimer2.Reset();
    myTimer2.Start();

}
void Intake::startTimer(){
    myTimer.Start(); 
    myTimer.Reset(); 
    sensorBool = false; 
    timeGotten = false; 
}
void Intake::setSolenoidTrue(){
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);

}
void Intake::setSolenoidFalse(){
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);

}

// This method will be called once per scheduler run
void Intake::Periodic(){
    units::time::second_t startTime = myTimer2.Get();
    bool intakeWorks = true; 
    if(p_conveyorMotor.GetFirmwareString() != firmwareVersion || p_intakeMotor.GetFirmwareString() != firmwareVersion){
        intakeWorks = false; 
    }
    frc::SmartDashboard::PutBoolean("Intake Working", intakeWorks); 
    conveyorVal = frc::SmartDashboard::GetNumber("Conveyor Value", .3); 
    frc::SmartDashboard::PutBoolean("ball sensor", p_intakeSensor.Get()); 

    units::time::second_t EndTime = myTimer2.Get();
    units::time::second_t RunTime= EndTime - startTime;
    if(0) {
        std::cout << "Intake Periodic Time: " << RunTime.value()<< std::endl;
    }

}

void Intake::IntakeBall(double setVal){
        p_intakeMotor.Set(setVal); 

    }




void Intake::StartConveyor(double percent){ 
    p_conveyorMotor.Set(percent); 
}
void Intake::resetOutBalls(){
    ballOut = 0; 
}
void Intake::PrepareBall(){
    if(!p_intakeSensor.Get() && ballCount < 3){
        if(!firstTimeGotten){
            firstTime = myTimer.Get(); 
            firstTimeGotten = true; 
        }
        if(myTimer.Get() - firstTime < 0.75_s){ //normal < .75, faster < .30, for power port < 0, test .50
            p_conveyorMotor.Set(0); 
        }
        
        else if(p_topSensor.Get()){
            // Original .375
            p_conveyorMotor.Set(.7); // was .55 
            sensorBool = true;  
        }
        
        
    } else if(p_intakeSensor.Get()){

            if(!timeGotten && sensorBool){
                
                conveyorTime = myTimer2.Get(); 
                firstTimeGotten = false; 
                timeGotten = true;
            }
            firstTime = 0_s; 
            myTimer.Reset();
            // Top Sensor 
            if(sensorBool){
                //Original .08
                if(myTimer2.Get() - conveyorTime < 0.15_s && ballCount < 3 && p_topSensor.Get()){ //conveyorTime was .25
                    //Original .375
                    p_conveyorMotor.Set(.7); //was .55
                }
                else{
                    sensorBool = false; 
                    timeGotten = false;
                    ballCount++; 
                }
                frc::SmartDashboard::PutNumber("Time - conveyorTime", (myTimer.Get() - conveyorTime).value()); 
            }
            else{
            p_conveyorMotor.Set(0);
            }
            //ballCount++;
        }
    if(p_topSensor.Get()){
        secondSensorBool = false; 
    }
    if(!p_topSensor.Get() && !secondSensorBool){
        ballOut++; 
    }

    

    
}

void Intake::ResetBallCount(){
    ballCount = 0; 
}

void Intake::StopMotors(){
    // p_conveyorMotor->Set(0);
    
    p_intakeMotor.Set(0);   
    
}