package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.SkyStone;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.opmode.OpModeTest.FOAM_TILE_INCH;

@Autonomous(group = "drive")
public class OpModeTest2 extends LinearOpMode {
    //public static double DISTANCE = 60;
    //public static double FOAM_TILE_INCH = 23.622;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap);

        //TODO STRATEGIE 1
        //Pornim cat sa vedem 3 cele mai din dr stonuri
        //Stim din init pozitia skystonurilor
        //Hardcodeala?

        //TODO STRATEGIE 2
        //Pornim cu tel in fata celui mai din dr stone
        //Mergem o anumita distanta in fata (1 FOAM_TILE_INCH)
        //Strafa stanga pana vad un skystone

        waitForStart();

        if (isStopRequested()) return;

        //STRATEGIE 2

//        while (opModeIsActive()){
//            telemetry.addData("found skystone", robot.drive.vuforiaLocalizer.isTargetVisible());
//            if(robot.drive.vuforiaLocalizer.isTargetVisible()) {
//                telemetry.addData("x", robot.drive.vuforiaLocalizer.getSkystoneOffset().getX());
//                telemetry.addData("y", robot.drive.vuforiaLocalizer.getSkystoneOffset().getY());
//                telemetry.addData("angle", robot.drive.vuforiaLocalizer.getSkystoneOffset().getHeading());
//            }
//            telemetry.update();
//        }



        //Position the robot
        robot.drive.getLocalizer().setPoseEstimate(new Pose2d(-1.5* FOAM_TILE_INCH, -2.5* FOAM_TILE_INCH, Math.toRadians(90)));

        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                                                .lineTo(new Vector2d(-1.5 *FOAM_TILE_INCH, -2*FOAM_TILE_INCH))
                                                .build());


        boolean found = false;

        //get skystone
        while (!found){
            found = robot.drive.vuforiaLocalizer.isTargetVisible();
            idle();
        }

        robot.drive.setMode(SampleMecanumDriveBase.Mode.IDLE);


        Vector2d absoluteSkystoneLocation = new Vector2d(robot.drive.vuforiaLocalizer.getSkystoneOffset().vec().getY() + robot.drive.getLocalizer().getPoseEstimate().vec().getX(),
                                                         -robot.drive.vuforiaLocalizer.getSkystoneOffset().vec().getX() + robot.drive.getLocalizer().getPoseEstimate().vec().getY());

        robot.drive.turnSync(Math.toRadians(90));

        Trajectory robotToSkystone = robot.drive.trajectoryBuilder()
                .strafeTo(absoluteSkystoneLocation)
                .build();

        //go to skystone
        robot.drive.followTrajectorySync(robotToSkystone);

        sleep(500);
        robot.skystoneArm.ArmDown();

        sleep(500);
        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder().strafeTo(new Vector2d(-1 * FOAM_TILE_INCH, -2 * FOAM_TILE_INCH)).build());
        robot.drive.turnSync(Math.toRadians(180));


        sleep(500);
        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.5*FOAM_TILE_INCH, -2*FOAM_TILE_INCH, Math.toRadians(0)))
                .build());

        robot.skystoneArm.ArmUp();
    }
}
