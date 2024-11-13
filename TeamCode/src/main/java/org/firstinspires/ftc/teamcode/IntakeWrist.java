
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeWrist {

    protected static DcMotor wristMotor;
    private final Gamepad gamepad; //if the wrist is automated, gamepad input might be unnecessary
    private final Telemetry telemetry;
    private final ElapsedTime runtime = new ElapsedTime();

    static final int LOW_HARDSTOP = 0;
    static final int HIGH_HARDSTOP = 125; // placeholder
    static final double MAX_SPEED = 0.2;
    // add statics as necessary

    static final int POSITION_TO_INTAKE = -117;
    static final int POSITION_TO_REST = -10;
    final boolean isAutonomous; // will not be necessary if wrist is automated

    public IntakeWrist(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {



        this.gamepad      = gamepad;
        this.isAutonomous = isAutonomous;
        this.telemetry    = telemetry;

        wristMotor = hardwareMap.get(DcMotor.class, "wristMotor"); // port 1
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wristMotor.setDirection(DcMotor.Direction.REVERSE);
        wristMotor.setTargetPosition(LOW_HARDSTOP);
        wristMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wristMotor.setPower(MAX_SPEED);

    }

    /**
     * utilizes DcMotor.Runmode.RUN_TO_POSITION to set the motor's target
     * @param ticks the distance the motor must run to, measured in ticks
     */
    public void setPosition(int ticks) {
        wristMotor.setTargetPosition(ticks);
    }

    private void readGamepad(Gamepad gamepad) {
       if(gamepad.dpad_up && IntakeSlide.intakeSlideMotor.getCurrentPosition() > 300) {
           wristMotor.setPower(MAX_SPEED);
           while (!(wristMotor.getCurrentPosition() >= POSITION_TO_REST)) {setPosition(POSITION_TO_REST);}
           wristMotor.setPower(0);
       }
       if(gamepad.dpad_down && IntakeSlide.intakeSlideMotor.getCurrentPosition() > 300) {
           wristMotor.setPower(MAX_SPEED);
           while (!(wristMotor.getCurrentPosition() <= POSITION_TO_INTAKE)) {setPosition(POSITION_TO_INTAKE);}
           wristMotor.setPower(0);
       }
       else if (gamepad.dpad_down && IntakeSlide.intakeSlideMotor.getCurrentPosition() < 300){
           IntakeSlide.intakeSlideMotor.setTargetPosition(350);
           wristMotor.setPower(MAX_SPEED);
           while (!(wristMotor.getCurrentPosition() <= POSITION_TO_INTAKE)) {setPosition(POSITION_TO_INTAKE);}
           wristMotor.setPower(0);
       }
    }


    public void loop() {
        if (!isAutonomous) readGamepad(gamepad);
        telemetry.addData("Wrist encoder count", wristMotor.getCurrentPosition());
    }


}
