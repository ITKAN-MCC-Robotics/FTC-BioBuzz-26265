package org.firstinspires.ftc.teamcode.utilities;

/**
 * Field coordinate definitions and conversion utilities for the BIOBUZZ season.
 * Defines FTC standard coordinates, converts to Pedro Pathing coordinates,
 * and maintains target coordinates for turret tracking.
 */
public class FieldCoordinates {

    // ---------------------------------------------------------
    // FTC Standard Coordinate System
    // Center of the field is (0,0). X is horizontal (parallel to audience).
    // Y is vertical (moving away from audience). Units are in inches.
    // ---------------------------------------------------------
    public static final double FIELD_CENTER_X = 0.0;
    public static final double FIELD_CENTER_Y = 0.0;

    // ---------------------------------------------------------
    // Starting & Reset Position Coordinates (Audience Wall, 2nd Tile)
    // Field width/height: 144" x 144" (-72" to +72" range).
    // Audience wall is at Y = -72.0".
    // Robot back against audience wall -> Center Y ≈ -63.0" (18" robot frame).
    // Tile 2 X center (Red side)  = -36.0" (2nd tile from left wall).
    // Tile 2 X center (Blue side) =  36.0" (2nd tile from right wall).
    // Default heading facing forward towards the field = 90.0 degrees.
    // ---------------------------------------------------------
    public static final double RED_START_X = -36.0;
    public static final double RED_START_Y = -63.0;
    public static final double RED_START_HEADING = 90.0;

    public static final double BLUE_START_X = 36.0;
    public static final double BLUE_START_Y = -63.0;
    public static final double BLUE_START_HEADING = 90.0;

    // Tile 2 exact center coordinates (if not touching wall)
    public static final double TILE_CENTER_Y = -60.0;

    // ---------------------------------------------------------
    // Targeting Coordinates (BIOBUZZ HIVE & Elements)
    // ---------------------------------------------------------
    // The HIVE structures serve as primary shooting targets for the turret.
    // Red Alliance HIVE target center (approximate based on standard layout)
    public static final double RED_HIVE_TARGET_X = -36.0; 
    public static final double RED_HIVE_TARGET_Y = 72.0;

    // Blue Alliance HIVE target center (approximate based on standard layout)
    public static final double BLUE_HIVE_TARGET_X = 36.0;
    public static final double BLUE_HIVE_TARGET_Y = 72.0;

    /**
     * Converts an FTC Standard X coordinate to a Pedro Pathing X coordinate.
     * Pedro Pathing sets (0,0) at the bottom corner, where X typically represents 
     * the forward/backward (vertical) axis relative to the starting wall.
     * 
     * @param ftcX The standard FTC X coordinate (inches).
     * @param ftcY The standard FTC Y coordinate (inches).
     * @return The Pedro Pathing X coordinate (inches).
     */
    public static double ftcToPedroX(double ftcX, double ftcY) {
        // In Pedro Pathing, X is the vertical axis extending from the alliance wall.
        return ftcY + 72.0;
    }

    /**
     * Converts an FTC Standard Y coordinate to a Pedro Pathing Y coordinate.
     * Pedro Pathing sets (0,0) at the bottom corner, where Y typically represents 
     * the left/right (horizontal) axis.
     * 
     * @param ftcX The standard FTC X coordinate (inches).
     * @param ftcY The standard FTC Y coordinate (inches).
     * @return The Pedro Pathing Y coordinate (inches).
     */
    public static double ftcToPedroY(double ftcX, double ftcY) {
        // In Pedro Pathing, Y is the horizontal axis. 
        // Adding 72 shifts the standard (-72 to 72) range to (0 to 144).
        return ftcX + 72.0;
    }

    /**
     * Calculates the required turret angle to face a target point on the field.
     * 
     * @param robotX Current robot X position in FTC standard coordinates.
     * @param robotY Current robot Y position in FTC standard coordinates.
     * @param targetX Target X position in FTC standard coordinates.
     * @param targetY Target Y position in FTC standard coordinates.
     * @return The angle in radians for the turret to aim at the target.
     */
    public static double calculateTurretAngle(double robotX, double robotY, double targetX, double targetY) {
        double deltaX = targetX - robotX;
        double deltaY = targetY - robotY;
        
        // Math.atan2 returns the angle in radians from -PI to PI
        return Math.atan2(deltaY, deltaX);
    }
}
