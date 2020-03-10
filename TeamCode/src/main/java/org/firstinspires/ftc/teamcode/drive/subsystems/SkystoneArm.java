package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SkystoneArm {
    private Servo armServo = null;

    public SkystoneArm(HardwareMap hardwareMap) {
        armServo = hardwareMap.servo.get("armServo");
        armServo.setPosition(0.7);
        armServo.setDirection(Servo.Direction.FORWARD);
    }

    public void ArmUp(){
        armServo.setPosition(1);
    }

    public void ArmDown(){
        armServo.setPosition(0);
    }
}
