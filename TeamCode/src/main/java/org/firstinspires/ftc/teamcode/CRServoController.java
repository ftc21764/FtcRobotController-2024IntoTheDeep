package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

public class CRServoController {
    private CRServo servo;

    public void init(HardwareMap hwMap, String deviceName) {
        servo = hwMap.get(CRServo.class, deviceName);
    }
    public void setPower(double power){
        servo.setPower(power);
    }

    /**
     * sets the servo's rotation direction. only needs to be set once.
     * @param direction uses com.qualcomm.robotcore.hardware.DcMotorSimple.Direction ,
     *                  either FORWARD or REVERSE
     *
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        servo.setDirection(direction);
    }
    public DcMotorSimple.Direction getServoDirection() {
        return servo.getDirection();
    }
    public double getServoPower() {
        return servo.getPower();
    }
}