// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

Intake::Intake() {
    stateIntake = 0;
    stateConveyor = 0;
    //m_loadIntoShooterMotor.Follow(m_conveyorMotor);
    frc::SmartDashboard::PutNumber("Intake Speed In", intakeSpeedIn);
    frc::SmartDashboard::PutNumber("Intake Speed Out", intakeSpeedOut);
    frc::SmartDashboard::PutNumber("Conveyor Speed In", conveyorSpeedFwd);
    frc::SmartDashboard::PutNumber("Conveyor Speed Out", conveyorSpeedBack);
    frc::SmartDashboard::PutNumber("Conveyor StopBallDistance", stopBallDistance);
    frc::SmartDashboard::PutNumber("Conveyor state", stateConveyor);
    frc::SmartDashboard::PutNumber("Intake state", stateIntake);
}

//lidar code
 
// Sensor 3 (ball sensor, D11 on Arduino)
float rrsDecoderBall(std::string inputArray){
        static float output = 0.0;
        std::string mmString = "";
        int mmInt = 0;
        // Finds the position of "B" marking the beginning of the range value
        int BIndex = inputArray.find("B");
        BIndex += 1; // Prevents "B" from being added to the return value
        // Adds on to cmString from inputArray until an "E" is reached
       
        // Converts mmInt string into int, converted to mm as a float
       if (inputArray == "" || inputArray == "B" || inputArray == "E" || inputArray == "BB" || inputArray == "BE"
      || inputArray == "," || inputArray == "b" || inputArray == "e"|| inputArray == "Z" || inputArray == "X"){
         mmString = "0";
       } else {
        while (inputArray[BIndex] != 'Z') {
            mmString += inputArray[BIndex];
            BIndex++;
        }
        }
        if (mmString == "64351"){
          output = INT_MAX;
        } else if(!(mmString == "0")) {
          //mmInt = stoi(mmString);
          std::stringstream ssString(mmString);
          ssString >> mmInt;
          output = mmInt/1000.0;
          //std::cout << output << std::endl;
        }
        //frc::SmartDashboard::PutNumber("Range 2", output);
        return output;
}
// Sensor 2 (right sensor, D10 on Arduino)
float rrsDecoderRight(std::string inputArray){
        static float output = 0.0;
        std::string mmString = "";
        int mmInt = 0;
        // Finds the position of "B" marking the beginning of the range value
        int BIndex = inputArray.find("R");
        BIndex += 1; // Prevents "B" from being added to the return value
        // Adds on to cmString from inputArray until an "E" is reached
       
        // Converts mmInt string into int, converted to mm as a float
       if (inputArray == "" || inputArray == "B" || inputArray == "E" || inputArray == "BB" || inputArray == "BE" || inputArray == "," || inputArray == "b" || inputArray == "e"){
         mmString = "0";
       } else {
        while (inputArray[BIndex] != 'b') {
            mmString += inputArray[BIndex];
            BIndex++;
        }
        }
        if (mmString == "64351"){
          output = INT_MAX;
        } else if(!(mmString == "0")) {
          //mmInt = stoi(mmString);
          std::stringstream ssString(mmString);
          ssString >> mmInt;
          output = mmInt/1000.0;
          //std::cout << output << std::endl;
        }
        //frc::SmartDashboard::PutNumber("Range 2", output);
        return output;
}

 

void Intake::SensorReset() {
    m_SerialMXP.SetTimeout(units::time::second_t(0.001));
    m_SerialMXP.SetReadBufferSize(18);
    m_SerialMXP.Reset();
}

//intake and conveyor code
// This method will be called once per scheduler run
void Intake::Periodic() {

    //lidar will need to be put back in
    char sSenseData[19] = {'\0'};
    int bytesRead = 0;
    //need to put back in to get data from sensor
    if (m_SerialMXP.GetBytesReceived() >= 36){
        bytesRead = m_SerialMXP.Read(sSenseData,18); 
        m_SerialMXP.Reset(); 
        SerialTime.Reset();
        SerialTime.Start();
    } /*else if (m_SerialMXP.GetBytesReceived() == 0 && timeSinceRead > 10.0){
        
    }*/
    //std::cout << "Serial data: " << sSenseData << std::endl;
    sSenseData[18] = '\0';
    std::string soSenseData = sSenseData;
    //frc::SmartDashboard::PutString("zzDJO Sense Data", soSenseData);
 
    //Sensor 3 (magazine)
    float fSenseData3 = rrsDecoderBall(soSenseData);

     if ((SerialTime.Get().value() > 1.0) || (autoConveyorFail == true)){
        fSenseData3 = 0.0;
        //autoConveyorFail = true;
    }
   
    frc::SmartDashboard::PutNumber("Intake Ball Range", fSenseData3);
    stopBallDistance = frc::SmartDashboard::GetNumber("Conveyor StopBallDistance", stopBallDistance);

    //std::cout << "Ball range: " << fSenseData3 << std::endl;


    //for tesing purposes only
    /*frc::SmartDashboard::PutBoolean("intakeSigIn",intakeSigIn);
    frc::SmartDashboard::PutBoolean("intakeSigInRelease", intakeSigInRelease);
    frc::SmartDashboard::PutBoolean("intakeSigOut", intakeSigOut);
    frc::SmartDashboard::PutBoolean("intakeSigOutReleasee", intakeSigOutRelease);
    frc::SmartDashboard::PutBoolean("conveyorSigFwd", conveyorSigFwd);
    frc::SmartDashboard::PutBoolean("conveyorSigFwdRelease", conveyorSigFwdReleaase);
    frc::SmartDashboard::PutBoolean("conveyorSigBack", conveyorSigBack);
    frc::SmartDashboard::PutBoolean("conveyorSigBackRelease", conveyorSigBackRelease);*/
    //frc::SmartDashboard::PutNumber("Conveyor Speed", m_conveyorMotor.Get());
    //frc::SmartDashboard::PutNumber("Intake Speed", m_intakeMotor.Get());
    frc::SmartDashboard::PutNumber("Conveyor state", stateConveyor);
    frc::SmartDashboard::PutNumber("Intake state", stateIntake);


    //conveyor state machine
    if (stateConveyor == 0){
        //initialization state
        conveyorSpeed = 0.0;
        m_conveyorMotor.Set(conveyorSpeed);
        stateConveyor = 3;
    } else if (stateConveyor == 1){
        //conveyor forward
        conveyorSpeedFwd = frc::SmartDashboard::GetNumber("Conveyor Speed In", conveyorSpeedFwd);
        m_conveyorMotor.Set(conveyorSpeedFwd); 
        conveyorSigFwd = false;

        if (conveyorSigFwdReleaase){
            stateConveyor = 3;
        } else if (conveyorSigBack){
            stateConveyor = 2;
        } 
    } else if (stateConveyor == 2){
        //conveyor backward
        conveyorSpeedBack = frc::SmartDashboard::GetNumber("Conveyor Speed Out", conveyorSpeedBack);
        m_conveyorMotor.Set(-conveyorSpeedBack); 
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

        if (conveyorSigFwd){
            stateConveyor = 1;
        } else if (conveyorSigBack){
            stateConveyor = 2;
        } /*else if (intakeSigIn && (fSenseData3 > stopBallDistance)) {
            stateConveyor = 4;
        } */
    } 
    /*else if (stateConveyor == 4){
        //conveyor forward that stops when ball reaches the shooter

        //takes input from lidar and stops the conveyor when a ball is detected
        if (fSenseData3 < stopBallDistance){
            //frc::SmartDashboard::PutString("Ball status", "ball ready");
            stateConveyor = 3;
        } else {
            //frc::SmartDashboard::PutString("Ball status", "ball not detected");
        }
        conveyorSpeed = 0.3;
        m_conveyorMotor.Set(conveyorSpeed); 
        conveyorSigFwd = false;

        if (conveyorSigFwdReleaase){
            stateConveyor = 3;
        } else if (conveyorSigBack){
            stateConveyor = 2; 
        } 
    }*/

    /*if (autoConveyor){
        //m_conveyorMotor.Set(0.1);
    } else {
        if (conveyorSigFwd){
            conveyorSpeedFwd = frc::SmartDashboard::GetNumber("Conveyor Speed In", conveyorSpeedFwd);
            m_conveyorMotor.Set(conveyorSpeedFwd);
        } else if (conveyorSigBack){
            conveyorSpeedBack = frc::SmartDashboard::GetNumber("Conveyor Speed Out", conveyorSpeedBack);
            m_conveyorMotor.Set(-conveyorSpeedBack);
        } else {
            m_conveyorMotor.Set(0.0);
        }
    }

    if (intakeSigIn){
        intakeSpeedIn = frc::SmartDashboard::GetNumber("Intake Speed In", intakeSpeedIn);
        m_intakeMotor.Set(-intakeSpeedIn);
    } else if (intakeSigOut){
        intakeSpeedOut = frc::SmartDashboard::GetNumber("Intake Speed Out", intakeSpeedOut);
        m_intakeMotor.Set(intakeSpeedOut);
    } else {
        m_intakeMotor.Set(0.0);
    }*/
    //intake state machine
    if (stateIntake == 0) {
        //initialization
        //reset timers
        intakeSpeed = 0.0;
        m_intakeMotor.Set(intakeSpeed); //also put in stateIntake 3
        //clean up ball counts or bools
        stateIntake = 3; //stopped stateIntake
    } else if (stateIntake == 1) {
        //Intake in with conveyor
        intakeSpeedIn = frc::SmartDashboard::GetNumber("Intake Speed In", intakeSpeedIn);
        m_intakeMotor.Set(-intakeSpeedIn); 
        intakeSigIn = false;
        //stateConveyor = 4; //starts conveyor motor

        //exit stateIntakes
        if(intakeSigInRelease) {
            stateIntake = 3;
        } else if (intakeSigOut) {
            stateIntake = 2;
        }
    } else if (stateIntake == 2) {
        //Intake out
        intakeSpeedOut = frc::SmartDashboard::GetNumber("Intake Speed Out", intakeSpeedOut);
        m_intakeMotor.Set(intakeSpeedOut);  
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

}

void Intake::IntakeIn(){
   intakeSigIn = true;
   //autoConveyor = true;
}
void Intake::IntakeInRelease(){
    intakeSigInRelease = true;
    //intakeSigIn = false;
}

void Intake::IntakeOut(){
    intakeSigOut = true;
}
void Intake::IntakeOutRelease(){
    intakeSigOutRelease = true;
    //intakeSigOut = false;
}

void Intake::ConveyorForward(){
    conveyorSigFwd = true;
    //autoConveyor = false;
}
void Intake::ConveyorForwardRelease(){
    conveyorSigFwdReleaase = true;
    //conveyorSigFwd = false;
}

void Intake::ConveyorBackward(){
    conveyorSigBack = true;
    //autoConveyor = false;
}
void Intake::ConveyorBackwardRelease(){
    conveyorSigBackRelease = true;
    //conveyorSigBack = false;
}