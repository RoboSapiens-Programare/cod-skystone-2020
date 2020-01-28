package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class RobotHardware extends LinearOpMode {
    protected DcMotor Encoder = null;

    public void initialize(){
        Encoder = hardwareMap.dcMotor.get("Encoder");
        Encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Encoder.setPower(0);
        Encoder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
