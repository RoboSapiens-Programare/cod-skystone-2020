package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDrive;

@Config
@Autonomous(group = "drive")
@Disabled

public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; //152

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new MecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .strafeLeft(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
