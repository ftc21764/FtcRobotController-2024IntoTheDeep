package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearLift {

    protected static DcMotor liftMotor;
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private final ElapsedTime runtime = new ElapsedTime();

    static final int LOW_HARDSTOP = 0;
    static final int HIGH_HARDSTOP = 0;
    static final double ADJUSTMENT_MODIFIER = 30;
    //todo: add more static variables like high middle and low positions

    //todo: determine what speed to set the motors to & whether up speed is different than down speed

    Boolean isAutonomous;

    public int targetPositionCount;

    public LinearLift(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {

        this.isAutonomous = isAutonomous;
        this.gamepad = gamepad;
        this.telemetry = telemetry;


        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor"); //port 2
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); //todo: figure out if we need a float or brake
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setTargetPosition(LOW_HARDSTOP);


        telemetry.addData("slide motor position", "%7d", liftMotor.getCurrentPosition());

    }

    /**
     * utilizes DcMotor.RunMode.RUN_TO_POSITION to set the motor's target
     *
     * @param amountOfTicksToMove what the motor must run to, measured in ticks
     */

    public void setPosition(int amountOfTicksToMove) {
        targetPositionCount = amountOfTicksToMove;
    }

    private void readGamepad(Gamepad gamepad) {
        if (gamepad.left_stick_y > 0.1 || gamepad.left_stick_y < -0.1 ) {

            targetPositionCount = Range.clip((int)(targetPositionCount + ADJUSTMENT_MODIFIER*-gamepad.left_stick_y), LOW_HARDSTOP, HIGH_HARDSTOP);

            telemetry.addData("Manual Branch", "Adjustment made");

        } else if (!liftMotor.isBusy()) {

            //This is so that if you let go of the joystick, it immediately stops the slide from moving. Not a bug!!!

            targetPositionCount = Range.clip((int) liftMotor.getCurrentPosition(), LOW_HARDSTOP, HIGH_HARDSTOP);
            liftMotor.setTargetPosition(targetPositionCount);

            telemetry.addData("Manual Branch", "Stop moving");
        } else {
            telemetry.addData("Manual Branch", "Running to Junction");

        }
    }

    public void loop() { //if linear slide doesnt run in auto, this parameter will be unnecessary
        if (!isAutonomous) {
            readGamepad(gamepad);
        }
        liftMotor.setTargetPosition(targetPositionCount);
        telemetry.addData("encoder position", liftMotor.getCurrentPosition());
    }

}