package org.firstinspires.ftc.teamcode.drive.opmode.driver;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.subsystems.Robot;

@Config
@TeleOp(group = "driver")
public class DriverMode extends OpMode {
    Robot robot = null;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, true);
    }

    @Override
    public void loop() {
        //robot.drive.setDriveSignal(); //TODO putem sa implementam si acceleratie daca suntem barosani

        //Practic baietii nostri au exact functia noastra de calculat vitezele
        robot.drive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));

        if(gamepad2.a){
            robot.skystoneArm.ArmDown();
        }
        else if(gamepad2.b){
            robot.skystoneArm.ArmUp();
        }

        if(gamepad2.x){
            robot.intakeMechanism.Intake();
        }
        else if (gamepad2.y){
            robot.intakeMechanism.Pushout();
        }
        else {
            robot.intakeMechanism.Stop();
        }
    }


}
