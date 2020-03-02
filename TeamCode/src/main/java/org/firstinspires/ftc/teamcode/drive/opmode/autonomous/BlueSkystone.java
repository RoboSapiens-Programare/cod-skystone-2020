package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "drive")
//@Disabled
public class BlueSkystone extends SkystoneAuto {

    public BlueSkystone() {
        super(true);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initSkystone();

        waitForStart();

        runSkystone();
    }
}
