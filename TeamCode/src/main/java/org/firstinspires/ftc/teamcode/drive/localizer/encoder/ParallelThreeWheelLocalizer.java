package org.firstinspires.ftc.teamcode.drive.localizer.encoder;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

@Config
public class ParallelThreeWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 20;
    public static double WHEEL_RADIUS = 2; // inch
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13; // inch; distance between the left and right wheels
    public static double FORWARD_OFFSET = 0; // inch; offset of the lateral wheel

    private DcMotor leftEncoder, rightEncoder, middleEncoder;

    public ParallelThreeWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = hardwareMap.dcMotor.get("LeftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("RightEncoder");
        middleEncoder = hardwareMap.dcMotor.get("MiddleEncoder");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(middleEncoder.getCurrentPosition())
        );
    }
}
