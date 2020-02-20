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

        //STRATEGIE 1

        //Position the robot
        robot.drive.getLocalizer().setPoseEstimate(new Pose2d(-1.5* FOAM_TILE_INCH, -2.5* FOAM_TILE_INCH, Math.toRadians(180)));

        //Array of found stones
        List<SkyStone> skystones = new ArrayList<>();



        //Wait until we get the position of the skystones or TODO: time has passed
        while(skystones.size() == 0){
            //do nothing
            telemetry.addData("label", robot.drive.tfodLocalizer.getLastTfodData().detectedObjectLabel);
            telemetry.addData("Recognitions", robot.drive.tfodLocalizer.tfod.getRecognitions().size());
            for(Recognition rec : robot.drive.tfodLocalizer.tfod.getRecognitions())
                telemetry.addData("rec ", rec.getLabel());

            telemetry.update();
            skystones = robot.drive.tfodLocalizer.getDetectedSkyStones();
            idle();
        }


        for(SkyStone skyStone : skystones) {
            //Create the trajectory to the skystones
            Trajectory pathToSkystone = robot.drive.trajectoryBuilder()
                    .strafeTo(skyStone.getRobotTargetLocation()) //TODO: maybe switch to spline
                    .build();



            robot.drive.followTrajectorySync(pathToSkystone);
            robot.drive.waitForIdle();

            sleep(500);

            //TODO: Dam bratul jos
            robot.skystoneArm.ArmDown();

            sleep(500);

            //Drag them to the building zone
            Trajectory pathToBuildingZone = robot.drive.trajectoryBuilder()
                    .splineTo(new Pose2d(1* FOAM_TILE_INCH, -2* FOAM_TILE_INCH, 0))
                    .build();

            robot.drive.followTrajectorySync(pathToBuildingZone);
            robot.drive.waitForIdle();

            //TODO: Dam bratul sus
            robot.skystoneArm.ArmUp();


        }



    }
}
