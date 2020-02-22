package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.SkyStone;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ROBOT_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.FieldConstants.FOAM_TILE_INCH;

@Autonomous(group = "drive")
public class OpModeTest extends LinearOpMode {
    //public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        telemetry.addData(">", "Initializing...");
        telemetry.update();

        while (robot.isInitializing()){
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        //STRATEGIE 2

        //Position the robot
        robot.drive.getLocalizer().setPoseEstimate(new Pose2d(-1.5* FOAM_TILE_INCH, -2.5* FOAM_TILE_INCH, Math.toRadians(90)));

        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                .lineTo(new Vector2d(-1.5 *FOAM_TILE_INCH, -1.7*FOAM_TILE_INCH), new LinearInterpolator(Math.toRadians(90), Math.toRadians(90)))
                .build());


        boolean found = false;

        //get skystone
        while (!found){
            found = robot.drive.vuforiaLocalizer.isTargetVisible();
            idle();
        }

        robot.drive.setMode(SampleMecanumDriveBase.Mode.IDLE);

        Vector2d absoluteSkystoneLocation = new Vector2d(robot.drive.vuforiaLocalizer.getSkystoneOffset().vec().getY() + robot.drive.getLocalizer().getPoseEstimate().vec().getX(),
                                                         -robot.drive.vuforiaLocalizer.getSkystoneOffset().vec().getX() + robot.drive.getLocalizer().getPoseEstimate().vec().getY() - (ROBOT_WIDTH / 2));

        Trajectory robotToSkystone = robot.drive.trajectoryBuilder()
                .strafeTo(absoluteSkystoneLocation)
                .build();

        //go to skystone
        robot.drive.followTrajectorySync(robotToSkystone);

        sleep(500);
        robot.skystoneArm.ArmDown();

        sleep(500);

        sleep(500);
        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.5*FOAM_TILE_INCH, -2*FOAM_TILE_INCH, Math.toRadians(0)), new ConstantInterpolator(Math.toRadians(180)))
                .build());

        robot.skystoneArm.ArmUp();
    }
}
