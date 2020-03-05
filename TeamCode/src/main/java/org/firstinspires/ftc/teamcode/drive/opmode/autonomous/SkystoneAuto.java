package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.drive.FieldConstants.FOAM_TILE_INCH;

@Autonomous(group = "drive")
public abstract class SkystoneAuto extends LinearOpMode {
    //Blue or red
    private int OPMODE_MIRROR;

    //Poses and vectors
    private Pose2d startPose;
    private Vector2d skystoneSearcherVec1;
    private Vector2d skystoneSearcherVec2;
    private Pose2d throughTheBridgePose;
    private Vector2d parkVec;

    //The robot
    private Robot robot;
    public static int MAX_MILISECONDS = 3000;


    public void initSkystone(boolean isBlue){
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);

        while (robot.isInitializing() && opModeIsActive()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        if(!isBlue){
            OPMODE_MIRROR = -1;
        }
        else{
            OPMODE_MIRROR = 1;
        }

        startPose = new Pose2d(-1.5 * FOAM_TILE_INCH, OPMODE_MIRROR * 2.6 * FOAM_TILE_INCH, Math.toRadians(180 + (OPMODE_MIRROR * 90)));
        skystoneSearcherVec1 = new Vector2d(-1.2 * FOAM_TILE_INCH, OPMODE_MIRROR * 1.9 * FOAM_TILE_INCH);
        skystoneSearcherVec2 = new Vector2d(-2.0 * FOAM_TILE_INCH, OPMODE_MIRROR * 1.9 * FOAM_TILE_INCH);
        throughTheBridgePose = new Pose2d(0.5 * FOAM_TILE_INCH, OPMODE_MIRROR * 2.0 * FOAM_TILE_INCH, Math.toRadians(180 + OPMODE_MIRROR * 150));
        parkVec = new Vector2d(-0.25, OPMODE_MIRROR * 1.5 * FOAM_TILE_INCH);

    }

    public void runSkystone() throws InterruptedException {
        //Position the robot
        robot.drive.getLocalizer().setPoseEstimate(startPose);

        //Go closer to the skystones
        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                .lineTo(skystoneSearcherVec1, new LinearInterpolator(startPose.getHeading(), Math.toRadians(-90)))
                .build());

        ///Get skystone
        boolean found = false;
        robot.timer.reset();

        //Analyze first 2 stones
        while (!found && robot.timer.milliseconds() <= MAX_MILISECONDS && opModeIsActive()){
            found = robot.vuforiaLocalizer.isSkystoneVisible();
            idle();
        }

        //If not found after some time, move to the next one
        if(!found){
            if(OPMODE_MIRROR == 1) {
                robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder().forward(7).build());
            }
            else{
                robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder().back(7).build());
            }
        }

        robot.timer.reset();

        //Try again
        while(!found && robot.timer.milliseconds() <= MAX_MILISECONDS && opModeIsActive()){
            found = robot.vuforiaLocalizer.isSkystoneVisible();
            idle();
        }

        //If found, proceed
        //If not, jump to next identification

        if(found) {
            sleep(200);

            Trajectory robotToSkystone = robot.drive.trajectoryBuilder()
                    .strafeTo(robot.vuforiaLocalizer.getSkystoneVec(robot.drive.getPoseEstimate()))
                    .build();

            //go to skystone
            robot.drive.followTrajectorySync(robotToSkystone);

            //get skystone
            sleep(500);
            robot.skystoneArm.ArmDown();

            sleep(500);

            Pose2d robotpose = robot.drive.getLocalizer().getPoseEstimate();

            //go under the bridge
            robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder(new Pose2d(robotpose.getX(), robotpose.getY(), Math.toRadians(OPMODE_MIRROR * 100)))
                    .splineTo(throughTheBridgePose, new ConstantInterpolator(robotpose.getHeading()))
                    .build());

            robot.skystoneArm.ArmUp();

            sleep(500);

        }

        //Go near the next skystones
        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                .lineTo(skystoneSearcherVec2, new ConstantInterpolator(robot.drive.getPoseEstimate().getHeading()))
                .build());


        found = false;
        sleep(1000);
        robot.timer.reset();

        //Analyze first 2 stones
        while (!found && robot.timer.milliseconds() <= MAX_MILISECONDS && opModeIsActive()){
            found = robot.vuforiaLocalizer.isSkystoneVisible();
            idle();
        }

        //If not found after some time, move to the next one
        if(!found){
            if(OPMODE_MIRROR == 1) {
                robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder().forward(7).build());
            }
            else{
                robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder().back(7).build());
            }
        }

        robot.timer.reset();

        //Try again
        while(!found && robot.timer.milliseconds() <= MAX_MILISECONDS && opModeIsActive()){
            found = robot.vuforiaLocalizer.isSkystoneVisible();
            idle();
        }

        //If found, proceed
        //If not, go under the bridge & park
        if(found) {
            sleep(200);

            Trajectory robotToSkystone = robot.drive.trajectoryBuilder()
                    .strafeTo(robot.vuforiaLocalizer.getSkystoneVec(robot.drive.getPoseEstimate()))
                    .build();

            //go to skystone
            robot.drive.followTrajectorySync(robotToSkystone);

            sleep(500);
            robot.skystoneArm.ArmDown();

            sleep(500);

        }
        Pose2d robotpose = robot.drive.getLocalizer().getPoseEstimate();

        //Go through under the bridge
        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder(new Pose2d(robotpose.getX(), robotpose.getY(), Math.toRadians(OPMODE_MIRROR * 100)))
                .splineTo(throughTheBridgePose, new ConstantInterpolator(robotpose.getHeading()))
                .build());

        robot.skystoneArm.ArmUp();

        //Park
        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder().lineTo(parkVec, new LinearInterpolator(robot.drive.getPoseEstimate().getHeading(), Math.toRadians(OPMODE_MIRROR * 90))).build());
    }
}
