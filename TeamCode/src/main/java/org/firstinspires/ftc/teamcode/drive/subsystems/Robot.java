package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    //TODO: clasa asta ar trebui sa aiba scop de logging si telemetry

    //Subsystems
    public MecanumDrive drive;
    public SkystoneArm skystoneArm;

    public Robot(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap);
        skystoneArm = new SkystoneArm(hardwareMap);
    }
}
