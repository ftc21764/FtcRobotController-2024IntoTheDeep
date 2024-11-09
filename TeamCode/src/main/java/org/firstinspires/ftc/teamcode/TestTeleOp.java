package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Tele-Op", group="Linear OpMode")
public class TestTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        Intake      intake      = new Intake(     hardwareMap, /*      */ gamepad1, false);
        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap, telemetry, gamepad2, false);
        IntakeWrist intakeWrist = new IntakeWrist(hardwareMap, telemetry, gamepad2, false);
        LinearLift  lift        = new LinearLift( hardwareMap, telemetry, gamepad2, false);
        Suspension  suspension  = new Suspension( hardwareMap, telemetry, gamepad2, false);

        telemetry.addLine("Test OpMode Initiated");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake.loop();
            intakeSlide.loop();
            intakeWrist.loop();
            lift.loop();
            suspension.loop();


//            telemetry.addData("Left Servo Position", intake.leftIntakeServo.getServoPosition());
//            telemetry.addData("Right Servo Position", intake.rightIntakeServo.getServoPosition());
        }
    }
}