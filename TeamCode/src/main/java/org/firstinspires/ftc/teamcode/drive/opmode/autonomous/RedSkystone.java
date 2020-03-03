package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;
import org.firstinspires.ftc.teamcode.drive.subsystems.SkystoneArm;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ROBOT_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.FieldConstants.FOAM_TILE_INCH;

@Autonomous(group = "drive")
public class RedSkystone extends SkystoneAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        initSkystone(false);

        waitForStart();

        runSkystone();
    }
}
