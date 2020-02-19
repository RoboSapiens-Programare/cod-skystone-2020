package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous(group = "drive")
public class OpModeTest2 extends LinearOpMode {
    //public static double DISTANCE = 60;
    public static double FOAM_TILE_CM = 23.622;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new MecanumDrive(hardwareMap);

        drive.getLocalizer().setPoseEstimate(new Pose2d(1.5 * FOAM_TILE_CM, 2.5 * FOAM_TILE_CM, Math.toRadians(-90)));




        waitForStart();

        if (isStopRequested()) return;

        List<String> Triggers = new ArrayList<>();
        Triggers.add("SkyStone");
        Triggers.add("SkyStone");

        drive.setTfodIdleTriggers(Triggers);
        drive.clearTfodIdleTriggers();

    }
}
