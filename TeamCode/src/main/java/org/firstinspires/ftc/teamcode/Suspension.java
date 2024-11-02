package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Suspension {

    protected static DcMotor suspensionMotor;
    private final Telemetry telemetry;
    private final Gamepad gamepad;


    private final ElapsedTime runtime = new ElapsedTime();

    static final int LOW_HARDSTOP = 0;
    static final int HIGH_HARDSTOP = 1000; // placeholder

    static final int ADJUSTMENT_MODIFIER = 30;

    final boolean isAutonomous;

    public int targetPositionCount;

    // todo: add more static encoder count variables as needed, like high, middle and low positions

    // todo: determine what speed to set the motors to & whether up speed is different than down speed

    public Suspension(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {
        // if linear slide doesn't run in auto, isAutonomous will be unnecessary

        suspensionMotor = hardwareMap.get(DcMotor.class,"liftMotor"); // port 2

        this.isAutonomous = isAutonomous;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        suspensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        suspensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // todo: figure out which value is best
        suspensionMotor.setDirection(DcMotor.Direction.FORWARD);
        suspensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        suspensionMotor.setTargetPosition(LOW_HARDSTOP);

        telemetry.addData("Slide motor position", "%7d", suspensionMotor.getCurrentPosition());

    }

    /**
     * utilizes DcMotor.Runmode.RUN_TO_POSITION to set the motor's target
     * @param ticks the distance the motor must run to, measured in ticks
     */
    public void setPosition(int ticks) {
        targetPositionCount = ticks;
    }

    private void readGamepad(Gamepad gamepad) {

        if (gamepad.left_stick_y > 0.1 || gamepad.left_stick_y < -0.1 ) {

            targetPositionCount = Range.clip((int)(targetPositionCount + ADJUSTMENT_MODIFIER*-gamepad.left_stick_y), LOW_HARDSTOP, HIGH_HARDSTOP);

            telemetry.addData("Manual Branch", "Adjustment made");

        } else if (!suspensionMotor.isBusy()) {

            //This is so that if you let go of the joystick, it immediately stops the arm from moving. Not a bug!!!

            targetPositionCount = Range.clip(suspensionMotor.getCurrentPosition(), LOW_HARDSTOP, HIGH_HARDSTOP);
            suspensionMotor.setTargetPosition(targetPositionCount);

            telemetry.addData("Manual Branch", "Stop moving");
        } else {
            telemetry.addData("Manual Branch", "Running to Junction");

        }
    }

    public void loop() {
        if (!isAutonomous) readGamepad(gamepad);
        suspensionMotor.setTargetPosition(targetPositionCount);
        telemetry.addData("Encoder position", suspensionMotor.getCurrentPosition());
    }
}