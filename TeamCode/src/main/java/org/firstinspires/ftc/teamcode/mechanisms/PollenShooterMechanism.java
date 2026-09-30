package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mechanism for the Pollen Shooter.
 */
public class PollenShooterMechanism {
    private DcMotorEx shooter1;
    private double targetRpm = 3000.0;

    /**
     * Initializes the shooter hardware.
     * @param hwMap The hardware map from the OpMode
     */
    public void init(HardwareMap hwMap) {
        shooter1 = hwMap.get(DcMotorEx.class, "Shooter1");

        // RUN_USING_ENCODER is required for velocity control
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets the target RPM for the shooter.
     * @param rpm The target RPM
     */
    public void setTargetRpm(double rpm) {
        this.targetRpm = rpm;
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
        this.targetRpm += amount;
        // Optional: constrain the RPM so it doesn't go negative
        if (this.targetRpm < 0) {
            this.targetRpm = 0;
        }
    }

    /**
     * Runs the shooter at the current target RPM.
     */
    public void runShooter() {
        // Convert RPM to degrees per second
        // 1 RPM = 360 degrees / 60 seconds
        double degreesPerSecond = targetRpm * (360.0 / 60.0);
        shooter1.setVelocity(degreesPerSecond, AngleUnit.DEGREES);
    }

    /**
     * Stops the shooter motor.
     */
    public void stopShooter() {
        // We can just set power to 0 to stop
        shooter1.setPower(0);
    }

    /**
     * Returns the current actual RPM of the shooter.
     * @return current RPM
     */
    public double getCurrentRpm() {
        if (shooter1 != null) {
            double degreesPerSecond = shooter1.getVelocity(AngleUnit.DEGREES);
            return degreesPerSecond * (60.0 / 360.0);
        }
        return 0.0;
    }
}
