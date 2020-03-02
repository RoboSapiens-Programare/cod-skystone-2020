package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathBuilder;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.profile.SimpleMotionConstraints;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.SkyStone;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ROBOT_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.FieldConstants.FOAM_TILE_INCH;

@Autonomous(group = "drive")
public class RedSkystone extends LinearOpMode {
    public static int MAX_MILISECONDS = 3000;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);

        while (robot.isInitializing()){
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        //Position the robot
        robot.drive.getLocalizer().setPoseEstimate(new Pose2d(-1.5* FOAM_TILE_INCH, -2.6* FOAM_TILE_INCH, Math.toRadians(90)));

        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                .lineTo(new Vector2d(-1.3 *FOAM_TILE_INCH, -2*FOAM_TILE_INCH), new LinearInterpolator(Math.toRadians(90), Math.toRadians(-90)))
                .build());

        //Get skystone
        robot.drive.setMode(SampleMecanumDriveBase.Mode.IDLE);

        boolean found = false;
        robot.timer.reset();

        while (!found && robot.timer.milliseconds() <= MAX_MILISECONDS){
            found = robot.vuforiaLocalizer.isTargetVisible();
            idle();
        }

        Vector2d absoluteSkystoneLocation;


        if(!found){
            robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder().forward(10).build());
        }

        while(!found){
            found = robot.vuforiaLocalizer.isTargetVisible();
            idle();
        }

        absoluteSkystoneLocation = new Vector2d(robot.vuforiaLocalizer.getSkystoneOffset().vec().getY() + robot.drive.getLocalizer().getPoseEstimate().vec().getX(),
                            -robot.vuforiaLocalizer.getSkystoneOffset().vec().getX() + robot.drive.getLocalizer().getPoseEstimate().vec().getY() - (ROBOT_WIDTH / 2));



        Trajectory robotToSkystone = robot.drive.trajectoryBuilder()
                .strafeTo(absoluteSkystoneLocation)
                .build();

        //go to skystone
        robot.drive.followTrajectorySync(robotToSkystone);

        sleep(500);
        robot.skystoneArm.ArmDown();

        sleep(500);

        Pose2d robotpose = robot.drive.getLocalizer().getPoseEstimate();

        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder(new Pose2d(robotpose.getX(), robotpose.getY(), Math.toRadians(250)))
                .splineTo(new Pose2d(0.5*FOAM_TILE_INCH, -2.2*FOAM_TILE_INCH, Math.toRadians(0)), new ConstantInterpolator(Math.toRadians(0)))
                .build());

        robot.skystoneArm.ArmUp();

        sleep (500);

        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                .lineTo(new Vector2d(-2*FOAM_TILE_INCH, -1.85*FOAM_TILE_INCH), new ConstantInterpolator(Math.toRadians(0)))
                .build());


        found = false;
        sleep(1000);
        robot.timer.reset();
        //robot.vuforiaLocalizer.clearSkystoneCache();

        while (!found && robot.timer.milliseconds() <= MAX_MILISECONDS){
            found = robot.vuforiaLocalizer.isTargetVisible();
            idle();
        }

        if(!found){
            robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder().back(14).build());
        }

        while(!found){
            found = robot.vuforiaLocalizer.isTargetVisible();
            idle();
        }

        absoluteSkystoneLocation = new Vector2d(robot.vuforiaLocalizer.getSkystoneOffset().vec().getY() + robot.drive.getLocalizer().getPoseEstimate().vec().getX(),
                    -robot.vuforiaLocalizer.getSkystoneOffset().vec().getX() + robot.drive.getLocalizer().getPoseEstimate().vec().getY() - (ROBOT_WIDTH / 2));



        robotToSkystone = robot.drive.trajectoryBuilder()
                .strafeTo(absoluteSkystoneLocation)
                .build();

        //go to skystone
        robot.drive.followTrajectorySync(robotToSkystone);

        sleep(500);
        robot.skystoneArm.ArmDown();

        sleep(500);

        robotpose = robot.drive.getLocalizer().getPoseEstimate();

        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder(new Pose2d(robotpose.getX(), robotpose.getY(), Math.toRadians(250)))
                .splineTo(new Pose2d(0.5*FOAM_TILE_INCH, -2*FOAM_TILE_INCH, Math.toRadians(0)), new ConstantInterpolator(Math.toRadians(0)))
                .build());

        robot.skystoneArm.ArmUp();

        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder().lineTo(new Vector2d(-0.25*FOAM_TILE_INCH, -1.5*FOAM_TILE_INCH), new LinearInterpolator(Math.toRadians(0), Math.toRadians(-90))).build());

    }
}
