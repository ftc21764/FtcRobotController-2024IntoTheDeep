package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    static final int HIGH_HARDSTOP = 3000;
    static final double ADJUSTMENT_MODIFIER = 30;
    static final double MAX_SPEED = 0.5;

    static final int HIGH_BUCKET = 3120;
    static final int LOW_BUCKET =  1850;

    //todo: determine what speed to set the motors to & whether up speed is different than down speed

    boolean isAutonomous;

    public int targetPositionCount;

    public LinearLift(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {

        this.isAutonomous = isAutonomous;
        this.gamepad = gamepad;
        this.telemetry = telemetry;


        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor"); //port 2
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //todo: figure out if we need a float or brake
        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setTargetPosition(LOW_HARDSTOP);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(MAX_SPEED);


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

            telemetry.addData("Manual Branch", "Stop moving");}
        else {
            telemetry.addData("Manual Branch", "Running to Junction");

        }
        if (gamepad.a){
            targetPositionCount = LOW_BUCKET;
        }
        if (gamepad.b){
           liftMotor.setPower(0);
        }
    }

    public void loop() { //if linear slide doesnt run in auto, this parameter will be unnecessary
        if (!isAutonomous) {
            readGamepad(gamepad);
        }
        liftMotor.setTargetPosition(targetPositionCount);
        telemetry.addData("Linear lift encoder position", liftMotor.getCurrentPosition());
        telemetry.addData("Linear lift target position", targetPositionCount);
    }

}
