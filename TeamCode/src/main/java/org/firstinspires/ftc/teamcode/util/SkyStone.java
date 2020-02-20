package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ROBOT_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.opmode.OpModeTest.FOAM_TILE_INCH;

public class SkyStone {
    public static final double LENGHT = 8.0;
    public static final double WIDTH = 4.0;
    public static final double ROBOT_TARGET_DIST = (WIDTH/2) + ROBOT_WIDTH;

    public Vector2d getLocation() {
        return location;
    }

    public Vector2d getRobotTargetLocation() {
        return targetLocation;
    }

    private Vector2d location;
    private Vector2d targetLocation;

    public SkyStone(int pos, boolean isBlue) {
        double stoneX, stoneY, targetY;
        stoneX = -(1*FOAM_TILE_INCH - LENGHT/2) - pos * LENGHT;
        stoneY = isBlue? FOAM_TILE_INCH - WIDTH : -FOAM_TILE_INCH + WIDTH ;

        targetY = isBlue? FOAM_TILE_INCH - (WIDTH - ROBOT_TARGET_DIST) : -FOAM_TILE_INCH + (WIDTH - ROBOT_TARGET_DIST);

        this.location = new Vector2d(stoneX, stoneY);
        this.targetLocation = new Vector2d(stoneX, targetY);
    }
}
