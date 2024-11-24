
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    static final double MAX_SPEED = 1;
    // add statics as necessary

    static final int POSITION_TO_INTAKE = -117;
    static final int POSITION_TO_DELIVER = -80;
    static final int POSITION_TO_REST = -10;
    final boolean isAutonomous; // will not be necessary if wrist is automated
    int wristTimer = 0;

    public IntakeWrist(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad, boolean isAutonomous) {



        this.gamepad      = gamepad;
        this.isAutonomous = isAutonomous;
        this.telemetry    = telemetry;

        wristMotor = hardwareMap.get(DcMotor.class, "wristMotor"); // port 1
        wristMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wristMotor.setDirection(DcMotor.Direction.FORWARD);
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

    public void WristUp() {
           wristMotor.setPower(MAX_SPEED);
           telemetry.addData("wrist motor speed", wristMotor.getPower());
           while (!(wristMotor.getCurrentPosition() >= POSITION_TO_REST)) {
               setPosition(POSITION_TO_REST);
               if (wristMotor.getCurrentPosition() > -40){
                   wristMotor.setPower(0.2);
                   telemetry.addData("Wrist motor speed", wristMotor.getPower());
               }
           }
           wristMotor.setPower(0);
       }

    public void WristDown(){

           IntakeSlide.intakeSlideMotor.setPower(.85);
           telemetry.addData("Wrist motor speed", wristMotor.getPower());
           while((IntakeSlide.intakeSlideMotor.getCurrentPosition() < 300)) {IntakeSlide.intakeSlideMotor.setTargetPosition(300);
           telemetry.addLine("In intake loop one");
           telemetry.update();}


           wristMotor.setPower(MAX_SPEED);
           while (!(wristMotor.getCurrentPosition() <= POSITION_TO_INTAKE) && wristTimer < 50) {
               telemetry.addLine("In intake loop 2");
               telemetry.update();
               setPosition(POSITION_TO_INTAKE);
               if (wristMotor.getCurrentPosition() < -40){
                   wristMotor.setPower(-0.1 );
                   telemetry.addData("Wrist motor speed", wristMotor.getPower());
                   wristTimer += 1;
                   telemetry.addData("wrist timer", wristTimer);
               }

           }
           telemetry.addLine("out of while loop");
           wristMotor.setPower(0);
           wristTimer = 0;
       }
    }


//    public void loop() {
//        if (!isAutonomous) readGamepad(gamepad);
//        telemetry.addData("Wrist encoder count", wristMotor.getCurrentPosition());
//        telemetry.addData("Random",Math.random());
//    }



