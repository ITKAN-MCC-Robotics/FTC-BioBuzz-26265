package org.firstinspires.ftc.teamcode.utilities;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field coordinate definitions and conversion utilities for the BIOBUZZ season.
 * Defines Pedro Pathing and FTC standard coordinates, converts between coordinate systems,
 * and maintains target coordinates for turret tracking.
 */
public class FieldCoordinates {

    // ---------------------------------------------------------
    // Pedro Pathing Starting & Reset Position Coordinates
    // Field dimensions: 141.5" x 141.5" (Origin 0,0 at bottom-left corner)
    // ---------------------------------------------------------
    // Red Alliance Start: Middle of 2nd tile by audience wall
    public static final double PEDRO_RED_START_X = 36.0;
    public static final double PEDRO_RED_START_Y = 9.0;
    public static final double PEDRO_RED_START_HEADING = 90.0; // Facing Up (+Y)

    // Blue Alliance Start: Middle of 2nd tile by opposite wall
    public static final double PEDRO_BLUE_START_X = 106.0;
    public static final double PEDRO_BLUE_START_Y = 132.0;
    public static final double PEDRO_BLUE_START_HEADING = 270.0; // Facing Down (-Y)

    // FTC Standard equivalents (Center at 0,0)
    public static final double FTC_RED_START_X = 63.0;
    public static final double FTC_RED_START_Y = -36.0;
    public static final double FTC_RED_START_HEADING = 180.0;

    public static final double FTC_BLUE_START_X = -60.0;
    public static final double FTC_BLUE_START_Y = 34.0;
    public static final double FTC_BLUE_START_HEADING = 0.0;

    // Field Y divider threshold (middle of field in Pedro coordinates)
    public static final double FIELD_MID_Y_PEDRO = 70.75;

    // ---------------------------------------------------------
    // Targeting Coordinates (BIOBUZZ HIVE Goals in Pedro Pathing Coordinates)
    // ---------------------------------------------------------
    // At Y < 70.75 (Lower / Audience half of field)
    public static final double PEDRO_RED_HIVE_LOWER_X = 70.75;
    public static final double PEDRO_RED_HIVE_LOWER_Y = 60.0;

    public static final double PEDRO_BLUE_HIVE_LOWER_X = 95.0;
    public static final double PEDRO_BLUE_HIVE_LOWER_Y = 60.0;

    // At Y >= 70.75 (Upper / Back half of field)
    public static final double PEDRO_RED_HIVE_UPPER_X = 70.75;
    public static final double PEDRO_RED_HIVE_UPPER_Y = 84.0;

    public static final double PEDRO_BLUE_HIVE_UPPER_X = 95.0;
    public static final double PEDRO_BLUE_HIVE_UPPER_Y = 84.0;

    /**
     * Gets the active target goal coordinates (X, Y) in Pedro Pathing coordinates based on
     * alliance and current robot Y position.
     *
     * @param isBlueAlliance true for Blue Alliance, false for Red Alliance
     * @param robotPedroY Current robot Y position in Pedro Pathing coordinates (inches)
     * @return double array containing [targetPedroX, targetPedroY]
     */
    public static double[] getTargetCoordinatesPedro(boolean isBlueAlliance, double robotPedroY) {
        if (robotPedroY < FIELD_MID_Y_PEDRO) {
            // Lower field (Y < 70.75)
            if (isBlueAlliance) {
                return new double[]{PEDRO_BLUE_HIVE_LOWER_X, PEDRO_BLUE_HIVE_LOWER_Y};
            } else {
                return new double[]{PEDRO_RED_HIVE_LOWER_X, PEDRO_RED_HIVE_LOWER_Y};
            }
        } else {
            // Upper field (Y >= 70.75)
            if (isBlueAlliance) {
                return new double[]{PEDRO_BLUE_HIVE_UPPER_X, PEDRO_BLUE_HIVE_UPPER_Y};
            } else {
                return new double[]{PEDRO_RED_HIVE_UPPER_X, PEDRO_RED_HIVE_UPPER_Y};
            }
        }
    }

    /**
     * Converts an FTC Standard X coordinate to a Pedro Pathing X coordinate.
     * Per Pedro Pathing specification: Pedro X = FTC Y + 72.
     */
    public static double ftcToPedroX(double ftcX, double ftcY) {
        return ftcY + 72.0;
    }

    /**
     * Converts an FTC Standard Y coordinate to a Pedro Pathing Y coordinate.
     * Per Pedro Pathing specification: Pedro Y = 72 - FTC X.
     */
    public static double ftcToPedroY(double ftcX, double ftcY) {
        return 72.0 - ftcX;
    }

    /**
     * Converts an FTC Standard Heading (radians) to Pedro Pathing Heading (radians).
     * Per Pedro Pathing specification: Pedro Heading = FTC Heading - PI / 2.
     */
    public static double ftcToPedroHeading(double ftcHeadingRad) {
        return ftcHeadingRad - (Math.PI / 2.0);
    }

    /**
     * Converts a Pedro Pathing X coordinate to an FTC Standard X coordinate.
     */
    public static double pedroToFtcX(double pedroX, double pedroY) {
        return 72.0 - pedroY;
    }

    /**
     * Converts a Pedro Pathing Y coordinate to an FTC Standard Y coordinate.
     */
    public static double pedroToFtcY(double pedroX, double pedroY) {
        return pedroX - 72.0;
    }

    /**
     * Converts a Pedro Pathing Heading (degrees) to FTC Standard Heading (degrees).
     */
    public static double pedroToFtcHeading(double pedroHeadingDeg) {
        return AngleUnit.normalizeDegrees(pedroHeadingDeg + 90.0);
    }

    /**
     * Calculates the absolute field angle in radians to face a target point from the robot's position.
     * Works identically in both FTC and Pedro coordinate systems.
     */
    public static double calculateTurretAngle(double robotX, double robotY, double targetX, double targetY) {
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        return Math.atan2(deltaY, deltaX);
    }

    /**
     * Calculates the relative angle in radians required for the turret to face a target point,
     * normalized relative to the current heading of the robot frame.
     */
    public static double calculateTurretAngleRelative(double robotX, double robotY, double robotHeadingRad, double targetX, double targetY) {
        double absTargetAngle = calculateTurretAngle(robotX, robotY, targetX, targetY);
        double relAngle = absTargetAngle - robotHeadingRad;
        return Math.atan2(Math.sin(relAngle), Math.cos(relAngle));
    }
}
