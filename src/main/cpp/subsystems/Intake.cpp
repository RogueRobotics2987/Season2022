// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() {
    stateIntake = 0;
    stateConveyor = 0;
}

//lidar code
float rrsDecoderFront(std::string inputArray) {
        static float output = 0.0;
        std::string mmString = "";
        int mmInt = 0;
        // Finds the position of "B" marking the beginning of the range value
        int BIndex = inputArray.find("B");
        BIndex += 1; // Prevents "B" from being added to the return value
        // Adds on to cmString from inputArray until an "E" is reached
       
        // Converts mmInt string into int, converted to mm as a float
       if (inputArray == "" || inputArray == "B" || inputArray == "E" || inputArray == "BB" || inputArray == "BE" || inputArray == "," || inputArray == "b" || inputArray == "e"){
         mmString = "0";
       } else {
        while (inputArray[BIndex] != 'E') {
            mmString += inputArray[BIndex];
            BIndex++;
        }
        }
        if(!(mmString == "0")) {
          mmInt = stoi(mmString);
          output = mmInt/100.0;
          std::cout << output << std::endl;
        }
        //frc::SmartDashboard::PutNumber("Range", output);
        return output;
}

float rrsDecoderBack(std::string inputArray){
        static float output = 0.0;
        std::string mmString = "";
        int mmInt = 0;
        // Finds the position of "B" marking the beginning of the range value
        int BIndex = inputArray.find("b");
        BIndex += 1; // Prevents "B" from being added to the return value
        // Adds on to cmString from inputArray until an "E" is reached
       
        // Converts mmInt string into int, converted to mm as a float
       if (inputArray == "" || inputArray == "B" || inputArray == "E" || inputArray == "BB" || inputArray == "BE" || inputArray == "," || inputArray == "b" || inputArray == "e"){
         mmString = "0";
       } else {
        while (inputArray[BIndex] != 'e') {
            mmString += inputArray[BIndex];
            BIndex++;
        }
        }
        if(!(mmString == "0")) {
          mmInt = stoi(mmString);
          output = mmInt/100.0;
          std::cout << output << std::endl;
        }
        //frc::SmartDashboard::PutNumber("Range 2", output);
        return output;
}

float rrsDecoderBall(std::string inputArray){
        static float output = 0.0;
        std::string mmString = "";
        int mmInt = 0;
        // Finds the position of "B" marking the beginning of the range value
        int BIndex = inputArray.find("Z");
        BIndex += 1; // Prevents "B" from being added to the return value
        // Adds on to cmString from inputArray until an "E" is reached
       
        // Converts mmInt string into int, converted to mm as a float
       if (inputArray == "" || inputArray == "B" || inputArray == "E" || inputArray == "BB" || inputArray == "BE"
      || inputArray == "," || inputArray == "b" || inputArray == "e"|| inputArray == "Z" || inputArray == "X"){
         mmString = "0";
       } else {
        while (inputArray[BIndex] != 'X') {
            mmString += inputArray[BIndex];
            BIndex++;
        }
        }
        if(!(mmString == "0")) {
          mmInt = stoi(mmString);
          output = mmInt/100.0;
          std::cout << output << std::endl;
        }
        //frc::SmartDashboard::PutNumber("Range 2", output);
        return output;
}
void Intake::SensorReset() {
    m_SerialMXP.SetTimeout(units::time::second_t(0.001));
    m_SerialMXP.SetReadBufferSize(5);
    m_SerialMXP.Reset();
}

//at some point create a reset function that can be called when we enter tele op

//solenoids no longer needed in intake subsystem, will be moved to climber
/*void Intake::setSolenoidTrue(){
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}
void Intake::setSolenoidFalse(){
    intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}*/

//intake and conveyor code
// This method will be called once per scheduler run
void Intake::Periodic() {

    //lidar
    /*char sSenseData[10] = {NULL};
    int bytesRead = 0;
    bytesRead = m_SerialMXP.Read(sSenseData,18);
    sSenseData[9] = NULL;
    std::string soSenseData = sSenseData;*/

    //Sensor 3 (magazine)
    //float fSenseData3 = rrsDecoderBack(soSenseData);
    //frc::SmartDashboard::PutNumber("Ball Range", fSenseData3);
    frc::SmartDashboard::PutBoolean("intakeSigIn",intakeSigIn);
    frc::SmartDashboard::PutBoolean("intakeSigInRelease", intakeSigInRelease);
    frc::SmartDashboard::PutBoolean("intakeSigOut", intakeSigOut);
    frc::SmartDashboard::PutBoolean("intakeSigOutReleasee", intakeSigOutRelease);
    frc::SmartDashboard::PutBoolean("conveyorSigFwd", conveyorSigFwd);
    frc::SmartDashboard::PutBoolean("conveyorSigFwdRelease", conveyorSigFwdReleaase);
    frc::SmartDashboard::PutBoolean("conveyorSigBack", conveyorSigBack);
    frc::SmartDashboard::PutBoolean("conveyorSigBackRelease", conveyorSigBackRelease);   
    frc::SmartDashboard::PutNumber("Intake speed", intakeSpeed);
    frc::SmartDashboard::PutNumber("Conveyor Speed", conveyorSpeed);
    frc::SmartDashboard::PutNumber("Conveyor Speed2", m_intakeMotor.Get());
  

    if (stateIntake == 0) {
        //initialization
        //reset timers
        intakeSpeed = 0.0;
        m_intakeMotor.Set(intakeSpeed); //also put in stateIntake 3
        //clean up ball counts or bools
        stateIntake = 3; //stopped stateIntake
    } else if (stateIntake == 1) {
        //Intake in with conveyor
        
        intakeSpeed = 0.3;
        m_intakeMotor.Set(intakeSpeed);  
        intakeSigIn = false;
        stateConveyor = 4; //starts conveyor motor

        //exit stateIntakes
        if(intakeSigInRelease) {
            stateIntake = 3;
        } else if (intakeSigOut) {
            stateIntake = 2;
        }
    } else if (stateIntake == 2) {
        //Intake out
        intakeSpeed = -0.3;
        m_intakeMotor.Set(intakeSpeed);
        intakeSigOut = false;
        
        //exit statements
        if (intakeSigOutRelease){
            stateIntake = 3;
        } else if (intakeSigIn) {
            stateIntake = 1;
        }
    } else if (stateIntake == 3){
        //stopped intake
        intakeSpeed = 0.0;
        m_intakeMotor.Set(intakeSpeed);
        intakeSigInRelease = false;
        intakeSigOutRelease = false;

        //exit statements 
        if (intakeSigIn) {
            stateIntake = 1;
        } else if (intakeSigOut){
            stateIntake = 2;
        }
    }

    //conveyor state machine
    if (stateConveyor == 0){
        //initialization state
        conveyorSpeed = 0.0;
        m_conveyorMotor.Set(conveyorSpeed);
        stateConveyor = 3;
    } else if (stateConveyor == 1){
        //conveyor forward
        conveyorSpeed = 0.3;
        m_conveyorMotor.Set(conveyorSpeed); 
        conveyorSigFwd = false;
        sensorDetectsBall = false;

        if (conveyorSigFwdReleaase){
            stateConveyor = 3;
        } else if (conveyorSigBack){
            stateConveyor = 2;
        } 
    } else if (stateConveyor == 2){
        //conveyor backward
        conveyorSpeed = -0.3;
        m_conveyorMotor.Set(conveyorSpeed);
        conveyorSigBack = false;

        if (conveyorSigBackRelease){
            stateConveyor = 3;
        } else if (conveyorSigFwd){
            stateConveyor = 1;
        }
    } else if (stateConveyor == 3){
        //conveyor moter stopped
        conveyorSpeed = 0.0;
        m_conveyorMotor.Set(conveyorSpeed);
        conveyorSigFwdReleaase = false;
        conveyorSigBackRelease = false;
        sensorDetectsBall = false;

        if (conveyorSigFwd){
            stateConveyor = 1;
        } else if (conveyorSigBack){
            stateConveyor = 2;
        }
    } else if (stateConveyor == 4){
        //conveyor forward that stops when ball reaches the shooter
        conveyorSpeed = 0.3;
        m_conveyorMotor.Set(conveyorSpeed); 
        conveyorSigFwd = false;

        /*if (fSenseData3 <= 0.2){
            //frc::SmartDashboard::PutString("Ball status", "ball ready");
            sensorDetectsBall = true;
        } */

        if (conveyorSigFwdReleaase){
            stateConveyor = 3;
        } else if (conveyorSigBack){
            stateConveyor = 2; 
        } else if (sensorDetectsBall){//ball detected
            stateConveyor = 3;
        }
    }



}

void Intake::IntakeIn(){
   // Set:ConveyorForward
   intakeSigIn = true;
}
void Intake::IntakeInRelease(){
    intakeSigInRelease = true;
}

void Intake::IntakeOut(){
    intakeSigOut = true;
    //m_intakeMotor.Set(0.5);
}
void Intake::IntakeOutRelease(){
    intakeSigOutRelease = true;
}

void Intake::ConveyorForward(){
    conveyorSigFwd = true;
}
void Intake::ConveyorForwardRelease(){
    conveyorSigFwdReleaase = true;
}

void Intake::ConveyorBackward(){
    conveyorSigBack = true;
}
void Intake::ConveyorBackwardRelease(){
    conveyorSigBackRelease = true;
}