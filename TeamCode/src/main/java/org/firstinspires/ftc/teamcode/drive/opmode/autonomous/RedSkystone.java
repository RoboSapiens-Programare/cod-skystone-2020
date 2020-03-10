package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(group = "drive")
public class RedSkystone extends SkystoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        initSkystone(false);

        waitForStart();

        runSkystone();
    }
}
