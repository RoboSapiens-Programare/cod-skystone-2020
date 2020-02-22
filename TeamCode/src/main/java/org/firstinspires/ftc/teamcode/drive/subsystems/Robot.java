package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    //TODO: clasa asta ar trebui sa aiba scop de logging si telemetry

    private boolean initializing;

    //Subsystems
    public MecanumDrive drive;
    public SkystoneArm skystoneArm;

    public Robot(HardwareMap hardwareMap) {
        initializing = true;

        drive = new MecanumDrive(hardwareMap);
        skystoneArm = new SkystoneArm(hardwareMap);

        initializing = false;
    }

    public boolean isInitializing() {
        return initializing;
    }
}
