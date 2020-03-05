package org.firstinspires.ftc.teamcode.drive.opmode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.firstinspires.ftc.teamcode.drive.FieldConstants.FOAM_TILE_INCH;

import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;

@Autonomous (group = "drive")
public abstract class ArinaSiAlecuSplineTo extends LinearOpMode {
            private Pose2d startpose;
            private Pose2d finishpose;

            private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, true);

        while (robot.isInitializing() && opModeIsActive()) {
            idle();
        }

        waitForStart();

        startpose = new Pose2d(-1.5 * FOAM_TILE_INCH ,  2.6 * FOAM_TILE_INCH ,Math.toRadians(-90 ));
        finishpose = new Pose2d( 1.7 * FOAM_TILE_INCH ,  0.3 * FOAM_TILE_INCH , Math.toRadians(-90));
        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder(startpose)
                .splineTo(finishpose, new ConstantInterpolator(robot.drive.getPoseEstimate().getHeading()))
                .build());

        robot.drive.waitForIdle();
    }


}
