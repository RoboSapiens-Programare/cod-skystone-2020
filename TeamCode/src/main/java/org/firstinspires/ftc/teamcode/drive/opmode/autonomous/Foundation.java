package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;

import static org.firstinspires.ftc.teamcode.drive.FieldConstants.FOAM_TILE_INCH;

@Autonomous(group = "drive")
public class Foundation extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        Robot robot = new Robot(hardwareMap);

        while (robot.isInitializing()) {
            idle();
        }

        telemetry.addData(">", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.followTrajectory(robot.drive.trajectoryBuilder().forward(1.5 * FOAM_TILE_INCH).build());
    }
}
