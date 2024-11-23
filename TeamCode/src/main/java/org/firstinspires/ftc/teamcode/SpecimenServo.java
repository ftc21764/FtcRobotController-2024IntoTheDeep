package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;


public class SpecimenServo {

//    private final Gamepad gamepad;

    ServoController specimenServo = new ServoController();

    static final double OPEN_POSITION = 0.5;
    static final double CLOSED_POSITION = 0.25;
//    final boolean isAutonomous;


    public SpecimenServo(HardwareMap hardwareMap){//, Gamepad gamepad, boolean isAutonomous){
//        this.gamepad = gamepad;
//        this.isAutonomous = isAutonomous;
        specimenServo.init(hardwareMap,"specimenServo");
    }
    public void CloseSpecimen (){
        specimenServo.setServoPosition(CLOSED_POSITION);
    }
    public void OpenSpecimen (){
        specimenServo.setServoPosition(OPEN_POSITION);
    }
//    public void loop(){
//        if(gamepad.x){
//            OpenSpecimen();
//        }
//        if (gamepad.y){
//            CloseSpecimen();
//        }
//    }
}
