package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Autoaim Mechanism for Biobuzz robot.
 * Controls the goBILDA turret run on "Turret1" mini axon servo.
 * Supports autoaim based on field position / side and manual adjustment.
 */
public class Autoaim {
    public enum FieldSide {
        LEFT,
        RIGHT
    }

    public static final double CENTER_POSITION = 0.5; // Straight forward zero position
    public static final double STEP_SIZE = 0.02;       // Manual step adjustment size
    public static final double LEFT_TARGET_POS = 0.65;  // Base autoaim pos for Left Field Side
    public static final double RIGHT_TARGET_POS = 0.35; // Base autoaim pos for Right Field Side

    private Servo turretServo;
    private FieldSide fieldSide = FieldSide.LEFT;
    private double manualOffset = 0.0;
    private double currentPosition = CENTER_POSITION;
    private boolean autoaimActive = false;

    /**
     * Initializes hardware for turret servo.
     * @param hwMap HardwareMap from OpMode
     */
    public void init(HardwareMap hwMap) {
        turretServo = hwMap.get(Servo.class, "Turret1");
        resetPosition();
    }

    /**
     * Resets turret position back to zero position (straight forward) and clears manual offset.
     */
    public void resetPosition() {
        manualOffset = 0.0;
        currentPosition = CENTER_POSITION;
        applyPosition(currentPosition);
    }

    /**
     * Incrementally step adjusts turret towards the right.
     */
    public void adjustRight() {
        manualOffset += STEP_SIZE;
        updateTurretPosition();
    }

    /**
     * Incrementally step adjusts turret towards the left.
     */
    public void adjustLeft() {
        manualOffset -= STEP_SIZE;
        updateTurretPosition();
    }

    /**
     * Sets field side for autoaim target targeting.
     * @param side FieldSide.LEFT or FieldSide.RIGHT
     */
    public void setFieldSide(FieldSide side) {
        this.fieldSide = side;
    }

    /**
     * Toggles field side between LEFT and RIGHT.
     */
    public void toggleFieldSide() {
        this.fieldSide = (fieldSide == FieldSide.LEFT) ? FieldSide.RIGHT : FieldSide.LEFT;
    }

    /**
     * Updates turret autoaim state and applies position to servo.
     * @param engageAutoaim true if autoaim sequence is engaged (Left Trigger pressed)
     */
    public void update(boolean engageAutoaim) {
        this.autoaimActive = engageAutoaim;
        updateTurretPosition();
    }

    /**
     * Calculates and applies final turret position considering autoaim mode and manual offset.
     */
    private void updateTurretPosition() {
        double basePosition;
        if (autoaimActive) {
            basePosition = (fieldSide == FieldSide.LEFT) ? LEFT_TARGET_POS : RIGHT_TARGET_POS;
        } else {
            basePosition = CENTER_POSITION;
        }

        currentPosition = Range.clip(basePosition + manualOffset, 0.0, 1.0);
        applyPosition(currentPosition);
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

    public double getManualOffset() {
        return manualOffset;
    }

    public boolean isAutoaimActive() {
        return autoaimActive;
    }

    public FieldSide getFieldSide() {
        return fieldSide;
    }
}
