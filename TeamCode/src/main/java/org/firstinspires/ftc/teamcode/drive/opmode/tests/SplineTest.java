package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled

public class SplineTest extends LinearOpMode {

    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot =  new Robot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(DISTANCE, 0, Math.toRadians(0)))
                .build());
    }
}
