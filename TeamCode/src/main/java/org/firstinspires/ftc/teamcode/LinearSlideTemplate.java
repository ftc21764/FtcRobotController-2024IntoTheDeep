package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class LinearSlideTemplate {

    protected static DcMotor slideMotor;
    private final Gamepad gamepad;
    private final ElapsedTime runtime = new ElapsedTime();

    static final int LOW_HARDSTOP = 0;
    static final int HIGH_HARDSTOP = 0;
    //todo: add more static variables like high middle and low positions

    //todo: determine what speed to set the motors to & whether up speed is different than down speed

    private int motorTickTarget = 0;
    Boolean isAutonomous;

    public LinearSlideTemplate(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {

        this.isAutonomous = isAutonomous;
        this.gamepad = gamepad;

        slideMotor = hardwareMap.get(DcMotor.class, "slide"); //placeholder name
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //todo: figure out if we need a float or brake
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setTargetPosition(LOW_HARDSTOP);


        telemetry.addData("slide motor position", "%7d", slideMotor.getCurrentPosition());

    }

    /**
     * utilizes DcMotor.RunMode.RUN_TO_POSITION to set the motor's target
     *
     * @param amountOfTicksToMove what the motor must run to, measured in ticks
     */

    public void setPosition(int amountOfTicksToMove) {
        slideMotor.setTargetPosition(amountOfTicksToMove);
        motorTickTarget = amountOfTicksToMove;
    }

    private void readGamepad(Gamepad gamepad) {
        /*this is where your controls go
        if(gamepad.a){setPosition(LOW_HARDSTOP)}
        if(gamepad.b){setPosition(HIGH_HARDSTOP)}*/
        if(gamepad.a){
            setPosition(HIGH_HARDSTOP);
        }
    }

    public void loop() { //if linear slide doesnt run in auto, this parameter will be unnecessary
        if (!isAutonomous) {
            readGamepad(gamepad);
        }

    }

}
