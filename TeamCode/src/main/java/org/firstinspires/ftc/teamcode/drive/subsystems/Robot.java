package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.localizer.vision.VuforiaThread;

public class Robot {
    //TODO: clasa asta ar trebui sa aiba scop de logging si telemetry

    private boolean initializing;

    //Subsystems
    public MecanumDrive drive;
    public SkystoneArm skystoneArm;
    public VuforiaThread vuforiaLocalizer;
    public IntakeMechanism intakeMechanism;

    //Utils
    public ElapsedTime timer;

    //Constructor with vuforia by default
    public Robot(HardwareMap hardwareMap) {
        initializing = true;

        drive = new MecanumDrive(hardwareMap);
        skystoneArm = new SkystoneArm(hardwareMap);
        vuforiaLocalizer = new VuforiaThread(hardwareMap);
        intakeMechanism = new IntakeMechanism(hardwareMap);

        vuforiaLocalizer.start();

        timer = new ElapsedTime();

        initializing = false;
    }

    //Constructor with vuforia option
    public Robot(HardwareMap hardwareMap, boolean suppressVuforia) {
        initializing = true;

        drive = new MecanumDrive(hardwareMap);
        skystoneArm = new SkystoneArm(hardwareMap);
        intakeMechanism = new IntakeMechanism(hardwareMap);

        if(!suppressVuforia) {
            vuforiaLocalizer = new VuforiaThread(hardwareMap);
            vuforiaLocalizer.start();
        }

        timer = new ElapsedTime();

        initializing = false;
    }

    public boolean isInitializing() {
        return initializing;
    }
}
