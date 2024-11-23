package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoController {
    private Servo servo;

    public void init(HardwareMap hwMap, String deviceName) {
        servo = hwMap.get(Servo.class, deviceName);
    }
    public void setServoPosition(double position){
        servo.setPosition(position);
    }
    public double getServoPosition() {
        return servo.getPosition();
    }
}