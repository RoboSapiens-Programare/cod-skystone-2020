package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanumsamples.SampleMecanumDriveREV;
import org.firstinspires.ftc.teamcode.drive.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.drive.FieldConstants.FOAM_TILE_INCH;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {

    public static double DISTANCE = 60;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot =  new Robot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectorySync(robot.drive.trajectoryBuilder()
                .splineTo(new Pose2d(DISTANCE, 0, Math.toRadians(0)))
                .build());
    }
}
