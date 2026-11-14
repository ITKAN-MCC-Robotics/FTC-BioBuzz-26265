package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utilities.FieldCoordinates;

/**
 * Autoaim Mechanism for Biobuzz robot.
 * Controls the goBILDA gear-driven turret using an Axon Mini MK2 servo ("turretP")
 * with analog position feedback encoder ("turretPfeedback").
 * 
 * Turret Specifications:
 * - Gear Ratio: 2.75:1
 * - Neutral / Straight Forward position: 0.5
 * - Expanded PWM Range: 500us - 2500us (300 degree servo stroke)
 * - Turret rotation limit: +/- 54.5 degrees from straight ahead (~109 degree total sweep)
 */
public class Autoaim {

    public static final double CENTER_POSITION = 0.5;        // Straight forward 0 deg position
    public static final double GEAR_RATIO = 2.75;            // goBILDA turret gear ratio (2.75:1)
    public static final double MAX_TURRET_LIMIT_DEG = 54.5;  // Max rotation limit in degrees (+/- 54.5 deg)
    public static final double SERVO_STANDARD_RANGE_DEG = 300.0; // Servo total angular sweep across 500us-2500us PWM range
    public static final boolean INVERT_SERVO_DIRECTION = false; // Set to true if positive angle turns clockwise

    private Servo turretServo;
    private AnalogInput turretEncoder;

    private double currentPosition = CENTER_POSITION;
    private double calculatedTargetAngleDeg = 0.0;
    private double clampedTurretAngleDeg = 0.0;
    private boolean isClamped = false;
    private boolean autoaimActive = false;
    private double manualOffset = 0.0;

    private double activeTargetX = 0.0;
    private double activeTargetY = 0.0;

    /**
     * Initializes hardware for turret servo ("turretP") and analog feedback encoder ("turretPfeedback").
     * Configures expanded 500us-2500us PWM range for maximum servo stroke.
     * @param hwMap HardwareMap from OpMode
     */
    public void init(HardwareMap hwMap) {
        // Look up servo hardware with fallback
        try {
            turretServo = hwMap.get(Servo.class, "turretP");
        } catch (Exception e1) {
            try {
                turretServo = hwMap.get(Servo.class, "Turret1");
            } catch (Exception e2) {
                turretServo = null;
            }
        }

        // Configure expanded PWM range (500us - 2500us) for 300 degree stroke
        if (turretServo instanceof ServoImplEx) {
            ((ServoImplEx) turretServo).setPwmRange(new PwmControl.PwmRange(500, 2500));
        }

        // Look up analog feedback encoder with fallback
        try {
            turretEncoder = hwMap.get(AnalogInput.class, "turretPfeedback");
        } catch (Exception e1) {
            try {
                turretEncoder = hwMap.get(AnalogInput.class, "turretEncoder");
            } catch (Exception e2) {
                turretEncoder = null;
            }
        }

        resetPosition();
    }

    /**
     * Resets turret position back to straight forward zero position (0.5).
     */
    public void resetPosition() {
        manualOffset = 0.0;
        calculatedTargetAngleDeg = 0.0;
        clampedTurretAngleDeg = 0.0;
        isClamped = false;
        autoaimActive = false;
        currentPosition = CENTER_POSITION;
        applyPosition(currentPosition);
    }

    /**
     * Updates turret autoaim state and servo output based on robot Pedro Pathing pose and alliance goal.
     * 
     * @param engageAutoaim true if autoaim trigger is held by driver (Left Trigger)
     * @param forceCenter true if driver manually forces center (Left Bumper)
     * @param robotPedroX Current robot Pedro Pathing X position in inches
     * @param robotPedroY Current robot Pedro Pathing Y position in inches
     * @param robotPedroHeadingRad Current robot Pedro Pathing heading in radians
     * @param isBlueAlliance true for Blue Alliance, false for Red Alliance
     */
    public void update(boolean engageAutoaim, boolean forceCenter,
                       double robotPedroX, double robotPedroY, double robotPedroHeadingRad,
                       boolean isBlueAlliance) {

        if (forceCenter) {
            // Left Bumper pressed: manually force turret back to center position (0.5)
            resetPosition();
            return;
        }

        this.autoaimActive = engageAutoaim;

        if (autoaimActive) {
            // Dynamic goal selection based on alliance and robot Pedro Y coordinate (< 70.75 vs >= 70.75)
            double[] targetCoords = FieldCoordinates.getTargetCoordinatesPedro(isBlueAlliance, robotPedroY);
            activeTargetX = targetCoords[0];
            activeTargetY = targetCoords[1];

            // Calculate relative angle to target in Pedro Pathing coordinates
            double deltaX = activeTargetX - robotPedroX;
            double deltaY = activeTargetY - robotPedroY;

            double absoluteTargetAngleRad = Math.atan2(deltaY, deltaX);
            double relativeAngleRad = Math.atan2(Math.sin(absoluteTargetAngleRad - robotPedroHeadingRad),
                                                 Math.cos(absoluteTargetAngleRad - robotPedroHeadingRad));

            calculatedTargetAngleDeg = Math.toDegrees(relativeAngleRad);

            // Apply +/- 50 degree soft limit constraint
            if (Math.abs(calculatedTargetAngleDeg) > MAX_TURRET_LIMIT_DEG) {
                isClamped = true;
                clampedTurretAngleDeg = Math.signum(calculatedTargetAngleDeg) * MAX_TURRET_LIMIT_DEG;
            } else {
                isClamped = false;
                clampedTurretAngleDeg = calculatedTargetAngleDeg;
            }

            // Convert target turret angle (degrees) to servo position
            // Servo angle required = turret angle * gear ratio
            double servoAngleDeg = clampedTurretAngleDeg * GEAR_RATIO;

            // Convert servo angle to normalized position delta around CENTER_POSITION (0.5)
            double positionDelta = servoAngleDeg / SERVO_STANDARD_RANGE_DEG;
            if (INVERT_SERVO_DIRECTION) {
                positionDelta = -positionDelta;
            }

            currentPosition = Range.clip(CENTER_POSITION + positionDelta + manualOffset, 0.0, 1.0);
        } else {
            // When auto-aim is not engaged, maintain center forward position + manual trim
            calculatedTargetAngleDeg = 0.0;
            clampedTurretAngleDeg = 0.0;
            isClamped = false;
            currentPosition = Range.clip(CENTER_POSITION + manualOffset, 0.0, 1.0);
        }

        applyPosition(currentPosition);
    }

    /**
     * Reads raw feedback voltage (0.0V to 3.3V) from the Axon Mini MK2 analog feedback wire.
     */
    public double getFeedbackVoltage() {
        return turretEncoder != null ? turretEncoder.getVoltage() : 0.0;
    }

    /**
     * Calculates actual physical turret angle in degrees from analog encoder feedback.
     */
    public double getFeedbackAngleDeg() {
        if (turretEncoder == null) return 0.0;
        double voltage = turretEncoder.getVoltage();
        double maxVoltage = turretEncoder.getMaxVoltage() > 0 ? turretEncoder.getMaxVoltage() : 3.3;
        double normalizedPos = voltage / maxVoltage;
        // 360 deg total encoder stroke divided by gear ratio gives physical turret angle
        return (normalizedPos - 0.5) * (360.0 / GEAR_RATIO);
    }

    /**
     * Incrementally step adjusts turret offset.
     * @param delta Position change step
     */
    public void adjustManualOffset(double delta) {
        manualOffset += delta;
    }

    /**
     * Applies position value directly to turret servo.
     * @param pos Servo position (0.0 to 1.0)
     */
    private void applyPosition(double pos) {
        if (turretServo != null) {
            turretServo.setPosition(pos);
        }
    }

    public double getTurretPosition() {
        return currentPosition;
    }

    public double getCalculatedTargetAngleDeg() {
        return calculatedTargetAngleDeg;
    }

    public double getClampedTurretAngleDeg() {
        return clampedTurretAngleDeg;
    }

    public double getActiveTargetX() {
        return activeTargetX;
    }

    public double getActiveTargetY() {
        return activeTargetY;
    }

    public boolean isClamped() {
        return isClamped;
    }

    public boolean isAutoaimActive() {
        return autoaimActive;
    }

    public double getManualOffset() {
        return manualOffset;
    }

    public boolean isEncoderAttached() {
        return turretEncoder != null;
    }

    public boolean isServoAttached() {
        return turretServo != null;
    }
}
