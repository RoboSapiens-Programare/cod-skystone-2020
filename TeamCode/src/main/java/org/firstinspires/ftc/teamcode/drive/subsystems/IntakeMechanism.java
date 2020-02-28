package org.firstinspires.ftc.teamcode.drive.subsystems;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeMechanism {
    private DcMotor leftIntakeMotor = null;
    private DcMotor rightIntakeMotor = null;

    public IntakeMechanism(HardwareMap hardwareMap) {
        leftIntakeMotor = hardwareMap.dcMotor.get("leftIntakeMotor"); //TODO rename
        rightIntakeMotor = hardwareMap.dcMotor.get("rightIntakeMotor"); //TODO rename

        leftIntakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void Intake() {
        leftIntakeMotor.setPower(0.7);
        rightIntakeMotor.setPower(0.7);
    }

    public void Pushout(){
        leftIntakeMotor.setPower(-0.7);
        rightIntakeMotor.setPower(-0.7);
    }

    public void Stop() {
        leftIntakeMotor.setPower(0);
        rightIntakeMotor.setPower(0);
    }
}
