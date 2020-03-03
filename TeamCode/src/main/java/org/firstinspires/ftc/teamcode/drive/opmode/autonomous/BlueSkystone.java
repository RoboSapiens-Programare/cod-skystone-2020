package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (group = "drive")
//@Disabled
public class BlueSkystone extends SkystoneAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        initSkystone(true);

        waitForStart();

        runSkystone();
    }
}
