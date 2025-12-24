package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceUtil {

    /**
     * The size of each tile in inches.
     */
    private static final double INCHES_PER_TILE = 23.625;

    /**
     * Converts a distance with arbitrary units to tiles.
     */
    public static double toTiles(double distance, DistanceUnit unit) {
        return inchesToTiles(DistanceUnit.INCH.fromUnit(unit, distance));
    }

    public static double tilesToInches(double distance) {
        return distance * INCHES_PER_TILE;
    }

    /**
     * Converts inches to tiles.
     */
    public static double inchesToTiles(double inches) {
        return inches / INCHES_PER_TILE;
    }

    public static double distanceBetween(SparkFunOTOS.Pose2D poseOne, SparkFunOTOS.Pose2D poseTwo){
        // c = sqrt((x2 - x1)^2 + (y2 - y1)^2)
        return Math.sqrt(Math.pow(poseTwo.x - poseOne.x, 2) + Math.pow(poseTwo.y - poseOne.y, 2));
    }

}
