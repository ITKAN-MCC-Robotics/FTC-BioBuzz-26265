package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.utilities.FieldCoordinates;

/**
 * GoBilda Pinpoint Odometry Computer Mechanism.
 * Handles hardware initialization, pod configuration, position tracking,
 * and integration with field target calculations.
 */
public class PinpointMechanism {

    private GoBildaPinpointDriver pinpoint;
    private boolean isInitialized = false;

    // Pod offsets (in millimeters) relative to the physical center of the robot
    // Robot dimensions: 18"x18" (457.2mm x 457.2mm), center is 228.6mm from back.
    // IMU location: centered width-wise (0mm), 130mm from back (-98.6mm from robot center).
    // X-pod (Forward pod): 60mm left of IMU -> +60.0 mm from robot center
    // Y-pod (Strafe pod): 20mm in front of IMU -> -98.6mm + 20mm = -78.6 mm from robot center
    private double xPodOffsetMm = 60.0;
    private double yPodOffsetMm = -78.6;

    /**
     * Initializes the Pinpoint Odometry Computer with default device name "pinpoint".
     * @param hwMap HardwareMap from OpMode
     */
    public void init(HardwareMap hwMap) {
        init(hwMap, "pinpoint");
    }

    /**
     * Initializes the Pinpoint Odometry Computer with a specified hardware map device name.
     * @param hwMap HardwareMap from OpMode
     * @param deviceName Hardware map device name
     */
    public void init(HardwareMap hwMap, String deviceName) {
        try {
            pinpoint = hwMap.get(GoBildaPinpointDriver.class, deviceName);

            // Configure default pod offsets
            pinpoint.setOffsets(xPodOffsetMm, yPodOffsetMm, DistanceUnit.MM);

            // Configure pod type (defaulting to goBILDA 4-Bar Pods)
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            // Set pod encoder count directions
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
            );

            // Reset position and calibrate IMU on startup
            pinpoint.resetPosAndIMU();
            isInitialized = true;
        } catch (Exception e) {
            isInitialized = false;
        }
    }

    /**
     * Sets the physical offsets of the odometry pods relative to the robot's tracking center.
     * @param xOffset X pod sideways offset (positive = left of tracking center)
     * @param yOffset Y pod forward offset (positive = forward of tracking center)
     * @param unit DistanceUnit (INCH or MM)
     */
    public void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        if (pinpoint != null) {
            pinpoint.setOffsets(xOffset, yOffset, unit);
        }
    }

    /**
     * Configures the odometry pod type resolution.
     * @param pods The goBILDA pod model (e.g. goBILDA_4_BAR_POD, goBILDA_SWINGARM_POD)
     */
    public void setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods pods) {
        if (pinpoint != null) {
            pinpoint.setEncoderResolution(pods);
        }
    }

    /**
     * Sets encoder counting directions.
     * @param xDirection X (forward) pod direction
     * @param yDirection Y (strafe) pod direction
     */
    public void setEncoderDirections(GoBildaPinpointDriver.EncoderDirection xDirection,
                                     GoBildaPinpointDriver.EncoderDirection yDirection) {
        if (pinpoint != null) {
            pinpoint.setEncoderDirections(xDirection, yDirection);
        }
    }

    /**
     * Updates the pinpoint position calculation. MUST be called once per OpMode loop cycle.
     */
    public void update() {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }

    /**
     * Sets a specific position and heading for field localization.
     * @param x X position in inches
     * @param y Y position in inches
     * @param headingDeg Heading in degrees
     */
    public void setPosition(double x, double y, double headingDeg) {
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, headingDeg));
        }
    }

    /**
     * Sets position using a Pose2D object.
     * @param pose Pose2D containing position and heading
     */
    public void setPosition(Pose2D pose) {
        if (pinpoint != null) {
            pinpoint.setPosition(pose);
        }
    }

    /**
     * Recalibrates the internal IMU and resets current field position.
     * Defaults to Red Alliance 2nd tile on audience wall (-36", -63", 90°).
     * Robot must be completely stationary during calibration.
     */
    public void resetPosAndIMU() {
        resetToAudienceWallRed();
    }

    /**
     * Recalibrates internal IMU and resets position to Red Alliance Start (Pedro: 36, 9, 90° | FTC: 63, -36, 180°):
     * Robot MUST be stationary during call.
     */
    public void resetToAudienceWallRed() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, FieldCoordinates.FTC_RED_START_X, FieldCoordinates.FTC_RED_START_Y, AngleUnit.DEGREES, FieldCoordinates.FTC_RED_START_HEADING));
        }
    }

    /**
     * Recalibrates internal IMU and resets position to Blue Alliance Start (Pedro: 106, 132, 270° | FTC: -60, 34, 0°):
     * Robot MUST be stationary during call.
     */
    public void resetToAudienceWallBlue() {
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, FieldCoordinates.FTC_BLUE_START_X, FieldCoordinates.FTC_BLUE_START_Y, AngleUnit.DEGREES, FieldCoordinates.FTC_BLUE_START_HEADING));
        }
    }

    /**
     * Gets current 2D field pose (Position and Heading).
     * @return Pose2D containing robot's position and orientation
     */
    public Pose2D getPose() {
        if (pinpoint != null) {
            return pinpoint.getPosition();
        }
        return new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }

    /**
     * Gets current X position in specified distance units.
     * @param unit DistanceUnit (INCH or MM)
     * @return X position
     */
    public double getX(DistanceUnit unit) {
        Pose2D pose = getPose();
        return pose != null ? pose.getX(unit) : 0.0;
    }

    /**
     * Gets current Y position in specified distance units.
     * @param unit DistanceUnit (INCH or MM)
     * @return Y position
     */
    public double getY(DistanceUnit unit) {
        Pose2D pose = getPose();
        return pose != null ? pose.getY(unit) : 0.0;
    }

    /**
     * Gets current heading angle in specified angle units.
     * @param unit AngleUnit (DEGREES or RADIANS)
     * @return Heading angle
     */
    public double getHeading(AngleUnit unit) {
        Pose2D pose = getPose();
        return pose != null ? pose.getHeading(unit) : 0.0;
    }

    /**
     * Converts FTC standard X position to Pedro Pathing X coordinate.
     * @return Pedro Pathing X position in inches
     */
    public double getPedroX() {
        return FieldCoordinates.ftcToPedroX(getX(DistanceUnit.INCH), getY(DistanceUnit.INCH));
    }

    /**
     * Converts FTC standard Y position to Pedro Pathing Y coordinate.
     * @return Pedro Pathing Y position in inches
     */
    public double getPedroY() {
        return FieldCoordinates.ftcToPedroY(getX(DistanceUnit.INCH), getY(DistanceUnit.INCH));
    }

    /**
     * Calculates turret angle required to target the Red Alliance target based on current Pedro Y position.
     * @return Target turret angle in radians
     */
    public double getAngleToRedTarget() {
        double[] target = FieldCoordinates.getTargetCoordinatesPedro(false, getPedroY());
        return FieldCoordinates.calculateTurretAngle(getPedroX(), getPedroY(), target[0], target[1]);
    }

    /**
     * Calculates turret angle required to target the Blue Alliance target based on current Pedro Y position.
     * @return Target turret angle in radians
     */
    public double getAngleToBlueTarget() {
        double[] target = FieldCoordinates.getTargetCoordinatesPedro(true, getPedroY());
        return FieldCoordinates.calculateTurretAngle(getPedroX(), getPedroY(), target[0], target[1]);
    }

    /**
     * Gets status string of the Pinpoint device.
     */
    public String getDeviceStatus() {
        if (pinpoint != null) {
            return pinpoint.getDeviceStatus().toString();
        }
        return "NOT INITIALIZED";
    }

    public boolean isInitialized() {
        return isInitialized;
    }
}
