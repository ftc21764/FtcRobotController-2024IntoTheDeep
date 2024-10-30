

/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.io.File;

/*
 * This file is heavily derived from the following samples; refer back to them for original source:
 *   - samples/BasicOmniOpMode_Linear.java (base class)
 *   - ftc21764/FtcRobotControllerPowerPlay/PowerPlayTeleop.java - for Field-Oriented driving
 *
 * Original Comment:
 *
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Into The Deep Tele-Op", group="Linear OpMode")
public class IntoTheDeepTeleOp extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // Note: The names here do not match the hardware names -- this should be fixed in the configuration.
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "right_driveB");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "right_driveF");
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "left_driveF");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "left_driveB");

        IntakeSlide intakeSlide = new IntakeSlide(hardwareMap, telemetry, gamepad2, false);
        LinearLift linearLift = new LinearLift(hardwareMap, telemetry, gamepad2, false);
        IntakeWrist intakeWrist = new IntakeWrist(hardwareMap, telemetry,gamepad2, false);

        // Initialize the IMU (Inertia Measurement Unit), used to detect the orientation of the robot
        // for Field-Oriented driving
        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )));


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.


        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        //swingArm.initLoop();
        //suspension.initLoop();
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            intakeSlide.loop();
            linearLift.loop();
            intakeWrist.loop();
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

            double y = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value ;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // Use the IMU to determine the orientation of the robot relative to its position when
            // initialized, and then calculate rotation
            //imu.getRobotOrientation()
            //double botHeading = -imu.getAngularOrientation().firstAngle;

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double botHeading = orientation.getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(botHeading) + y * Math.sin(botHeading);
            double rotY = -x * Math.sin(botHeading) + y * Math.cos(botHeading);

            /*
            Fixed(?) drivetrain weirdness.

            Serious (old) code:
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            Silly (current) code:
            double rotX = x * Math.cos(botHeading) + y * Math.sin(botHeading);
            double rotY = -x * Math.sin(botHeading) + y * Math.cos(botHeading);

             */

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = rotY + rotX + rx;
            double rightFrontPower = rotY - rotX - rx;
            double leftBackPower = rotY - rotX + rx;
            double rightBackPower = rotY + rotX - rx;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Hold the left bumper and the corresponding button to run test code.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            if (gamepad1.left_bumper) {
                leftFrontPower = gamepad1.x ? 1.0 : 0.0;  // X gamepad
                leftBackPower = gamepad1.a ? 1.0 : 0.0;   // A gamepad
                rightFrontPower = gamepad1.y ? 1.0 : 0.0; // Y gamepad
                rightBackPower = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            }

            double speedModifier;
            if (!(gamepad1.left_stick_button || gamepad1.right_bumper)) {
                speedModifier = 0.5;
            } else {
                speedModifier = 1;
            }

            if (gamepad1.back) {
                gamepad1.rumble(100);
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, new File("C:\\Users\\TekersRobotics\\StudioProjects\\FtcRobotController-2023CenterStage\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\Alert.mp3"));
                imu.resetYaw();
            }

            if(orientation.getYaw(AngleUnit.RADIANS) == 0.0) {
                gamepad1.rumble(250);
            }

            // Send calculated power to wheels
            frontLeftDrive.setPower(leftFrontPower * speedModifier);
            frontRightDrive.setPower(rightFrontPower * speedModifier);
            backLeftDrive.setPower(leftBackPower * speedModifier);
            backRightDrive.setPower(rightBackPower * speedModifier);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addLine("LB + A/B/X/Y to test single motors");
            telemetry.addLine("");
            telemetry.addData("IMU orientation", botHeading);
            telemetry.addLine("");
            //telemetry.addData("Door servo position", doorServo.doorServo.getServoPosition());
            telemetry.update();
        }
    }
}
/* Pseudocode: IMU reset edition

Option 1:
while opModeIsActive {

    heading = imu.heading + crashCorrection

    if !(imu.heading == 0.0) {
        crashBackup = imu.heading
    }

    if imu.heading == 0.0 {
        crashCorrection = crashBackup
        imu.reset
    }

Option 2:
-Wait until IMU resets
-Notify drivers with controller vibration
-Allow drivers to manually reset position
-Allow drivers to press a button to reset IMU

Example:

while opModeIsActive {

    if String(imu.heading) == "0.0" {
        gamepad.rumble(0.1);
    }

    if gamepad.start {
    */
