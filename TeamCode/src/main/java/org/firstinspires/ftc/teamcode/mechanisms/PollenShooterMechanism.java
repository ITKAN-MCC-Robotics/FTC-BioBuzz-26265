package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mechanism for the Pollen Shooter.
 * Manages the Shooter1 flywheel motor, velocity setpoints, speed permissives,
 * and flywheel toggling.
 */
public class PollenShooterMechanism {
    public static final double DEFAULT_RPM = 2800.0;
    public static final double MAX_RPM = 4500.0;
    public static final double RPM_TOLERANCE = 100.0; // Allowed error for speed permissive

    private DcMotorEx shooter1;
    private double targetRpm = DEFAULT_RPM;
    private boolean flywheelEnabled = false;

    /**
     * Initializes the shooter hardware.
     * @param hwMap The hardware map from the OpMode
     */
    public void init(HardwareMap hwMap) {
        shooter1 = hwMap.get(DcMotorEx.class, "Shooter1");

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Toggles the flywheel state on or off.
     */
    public void toggleFlywheel() {
        setFlywheelOn(!flywheelEnabled);
    }

    /**
     * Enables or disables the flywheel.
     * @param enabled true to enable, false to disable
     */
    public void setFlywheelOn(boolean enabled) {
        this.flywheelEnabled = enabled;
        if (!flywheelEnabled) {
            stopShooter();
        }
    }

    /**
     * Returns whether the flywheel is enabled.
     * @return true if enabled
     */
    public boolean isFlywheelOn() {
        return flywheelEnabled;
    }

    /**
     * Sets the target RPM for the shooter.
     * @param rpm The target RPM
     */
    public void setTargetRpm(double rpm) {
        this.targetRpm = Range.clip(rpm, 0.0, MAX_RPM);
    }

    /**
     * Returns the current target RPM.
     * @return target RPM
     */
    public double getTargetRpm() {
        return this.targetRpm;
    }

    /**
     * Adjusts the current target RPM by the specified amount.
     * @param amount the RPM to add or subtract
     */
    public void adjustRpm(double amount) {
        setTargetRpm(this.targetRpm + amount);
    }

    /**
     * Runs the shooter at the current target RPM using encoder velocity control.
     */
    public void runShooter() {
        if (shooter1 != null) {
            // Convert RPM to degrees per second: 1 RPM = 360 deg / 60 sec = 6 deg/sec
            double degreesPerSecond = targetRpm * 6.0;
            shooter1.setVelocity(degreesPerSecond, AngleUnit.DEGREES);
        }
    }

    /**
     * Stops the shooter motor.
     */
    public void stopShooter() {
        if (shooter1 != null) {
            shooter1.setPower(0.0);
        }
    }

    /**
     * Updates the shooter execution based on flywheel status.
     */
    public void update() {
        if (flywheelEnabled) {
            runShooter();
        } else {
            stopShooter();
        }
    }

    /**
     * Returns the current actual RPM of the shooter.
     * @return current RPM
     */
    public double getCurrentRpm() {
        if (shooter1 != null) {
            double degreesPerSecond = shooter1.getVelocity(AngleUnit.DEGREES);
            return degreesPerSecond / 6.0;
        }
        return 0.0;
    }

    /**
     * Checks if the flywheel has reached target speed (speed permissive).
     * @return true if actual RPM is within RPM_TOLERANCE of target RPM.
     */
    public boolean isAtTargetSpeed() {
        if (targetRpm <= 0) return true;
        double currentRpm = getCurrentRpm();
        return Math.abs(currentRpm - targetRpm) <= RPM_TOLERANCE || currentRpm >= (targetRpm - RPM_TOLERANCE);
    }
}
