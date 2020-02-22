package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDrive;

@Config
@Autonomous(group = "drive")
public class InterpolatorTest extends LinearOpMode {
    public static double DISTANCE = 60; //152

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new MecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder()
                .lineTo(new Vector2d(DISTANCE, 0), new LinearInterpolator(Math.toRadians(0),Math.toRadians(90)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);
    }
}
