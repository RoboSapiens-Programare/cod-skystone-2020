package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import static org.firstinspires.ftc.teamcode.drive.opmode.OpModeTest2.FOAM_TILE_CM;

public class SkyStone {
    public static final double LENGHT = 8.0;
    public static final double WIDTH = 4.0;

    public Vector2d getLocation() {
        return location;
    }

    private Vector2d location;

    public SkyStone(int pos, boolean isBlue) {
        pos = isBlue? pos : pos > 2? pos - 6 : pos;
        pos = (isBlue? 1 : 3) + ((isBlue? 1 : -1) * pos);

        double stoneX, stoneY;
        stoneX = -(1*FOAM_TILE_CM - LENGHT/2) + pos * LENGHT;
        stoneY = isBlue? 1*FOAM_TILE_CM - WIDTH : -1*FOAM_TILE_CM + WIDTH;

        this.location = new Vector2d(stoneX, stoneY);
    }
}
