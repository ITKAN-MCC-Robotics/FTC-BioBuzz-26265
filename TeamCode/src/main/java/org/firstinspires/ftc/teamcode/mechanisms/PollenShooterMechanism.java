package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

/**
 * Mechanism for the Pollen Shooter.
 * Uses closed-loop PIDF velocity control based on motor encoder specifications.
 */
public class PollenShooterMechanism {
    public static final double DEFAULT_RPM = 2900.0;
    public static final double MAX_RPM = 4000.0;
    public static final double TICKS_PER_REV = 28.0; // 28 TPR for goBILDA / REV 1:1 motor
    public static final double RPM_TOLERANCE = 180.0; // Speed permissive tolerance

    private DcMotorEx shooterP;
    private double targetRpm = DEFAULT_RPM;
    private boolean flywheelEnabled = false;

    /**
     * Initializes the shooter hardware and configures closed-loop PIDF velocity control.
     * @param hwMap The hardware map from the OpMode
     */
    public void init(HardwareMap hwMap) {
        // Hardware lookup with fallbacks for motor name
        shooterP = getMotorWithFallback(hwMap, "shooterP", "shooter");

        if (shooterP != null) {
            // Set direction for shooter launch
            shooterP.setDirection(DcMotorSimple.Direction.REVERSE);
            shooterP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooterP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // Configure PIDF feedforward & proportional gain for closed-loop velocity control
            // Max Ticks/Sec = (6000 RPM * 28 TPR) / 60 = 2800 ticks/sec
            double maxTicksPerSecond = (MAX_RPM * TICKS_PER_REV) / 60.0;
            double f = 20000 / maxTicksPerSecond;
            double p = 10.0;
            double i = 1.0;
            double d = 0.0;

            shooterP.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
        }
    }

    private DcMotorEx getMotorWithFallback(HardwareMap hwMap, String... names) {
        for (String name : names) {
            try {
                DcMotorEx m = hwMap.get(DcMotorEx.class, name);
                if (m != null) return m;
            } catch (Exception ignored) {
            }
        }
        return null;
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
     * Runs the shooter at target RPM using closed-loop encoder velocity control (ticks/sec).
     */
    public void runShooter() {
        if (shooterP != null) {
            // Convert RPM to encoder ticks per second: (RPM * 28 TPR) / 60 sec
            double targetTicksPerSec = targetRpm * (TICKS_PER_REV / 60.0);
            shooterP.setVelocity(targetTicksPerSec);
        }
    }

    /**
     * Stops the shooter motor.
     */
    public void stopShooter() {
        if (shooterP != null) {
            shooterP.setPower(0.0);
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
     * Returns actual RPM magnitude calculated from encoder velocity.
     * @return current RPM
     */
    public double getCurrentRpm() {
        if (shooterP != null) {
            double actualTicksPerSec = Math.abs(shooterP.getVelocity());
            return actualTicksPerSec * (60.0 / TICKS_PER_REV);
        }
        return 0.0;
    }

    /**
     * Returns raw velocity in encoder ticks per second.
     * @return ticks/sec
     */
    public double getVelocityTicksPerSec() {
        return shooterP != null ? Math.abs(shooterP.getVelocity()) : 0.0;
    }

    /**
     * Returns raw encoder ticks position.
     * @return encoder position ticks
     */
    public int getEncoderPosition() {
        return shooterP != null ? shooterP.getCurrentPosition() : 0;
    }

    /**
     * Checks if actual RPM is within speed permissive tolerance (100 RPM).
     * @return true if speed setpoint reached within tolerance or shooter hardware disabled
     */
    public boolean isAtTargetSpeed() {
        if (shooterP == null || targetRpm <= 0) return true;
        double currentRpm = getCurrentRpm();
        return Math.abs(currentRpm - targetRpm) <= RPM_TOLERANCE;
    }
}
