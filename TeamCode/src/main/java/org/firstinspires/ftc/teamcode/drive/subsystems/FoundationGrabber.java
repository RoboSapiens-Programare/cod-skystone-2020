package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class FoundationGrabber {
    private Servo servoLeft;
    private Servo servoRight;

    public FoundationGrabber(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.servo.get("servoLeft");
        servoRight = hardwareMap.servo.get("servoRight");

        servoLeft.setPosition(0);
        servoRight.setPosition(1);
    }

    public void Grab(){
        servoLeft.setPosition(1);
        servoRight.setPosition(0);
    }

    public void Release(){
        servoLeft.setPosition(0);
        servoRight.setPosition(1);
    }
}
