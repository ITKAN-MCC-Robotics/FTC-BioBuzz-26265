package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

/**
 * shooterTestOpMode - TeleOp OpMode for testing a 6000 RPM shooter motor
 * with live dynamic PIDF parameter tuning.
 * <p>
 * Controls:
 * - Gamepad A             : Start motor at target RPM
 * - Gamepad B             : Stop motor
 * - Gamepad X             : Reset PIDF coefficients to defaults
 * - D-Pad Alone           : Up/Down +/- 100 RPM, Right/Left +/- 500 RPM
 * - Hold Left Bumper      : D-Pad Up/Down +/- 1.0 P, Right/Left +/- 0.1 P
 * - Hold Right Bumper     : D-Pad Up/Down +/- 0.5 F, Right/Left +/- 0.05 F
 */
@TeleOp(name = "shooterTestOpMode", group = "Testing")
public class shooterTestOpMode extends OpMode {

    private DcMotorEx shooterP;

    // 6000 RPM Motor Specs (28 ticks per revolution for goBILDA/REV 1:1 bare motor)
    private static final double MAX_RPM = 6000.0;
    private static final double TICKS_PER_REV = 28.0;

    // Default PIDF coefficients
    // Max Ticks/Sec = (6000 RPM * 28 TPR) / 60 = 2800 ticks/sec
    // Feedforward F = 32767 / 2800 ≈ 11.7
    private static final double DEFAULT_P = 10.0;
    private static final double DEFAULT_I = 1.0;
    private static final double DEFAULT_D = 0.0;
    private static final double DEFAULT_F = 32767.0 / ((MAX_RPM * TICKS_PER_REV) / 60.0);

    // Live adjustable PIDF gains
    private double p = DEFAULT_P;
    private double i = DEFAULT_I;
    private double d = DEFAULT_D;
    private double f = DEFAULT_F;

    private double targetRpm = 3000.0; // Default starting target RPM
    private boolean isRunning = false;

    // Previous button states for edge detection
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadRight = false;
    private boolean previousDpadLeft = false;
    private boolean previousX = false;

    @Override
    public void init() {
        // 1. Hardware Map lookup for motor "shooter"
        shooterP = hardwareMap.get(DcMotorEx.class, "shooter");

        // 2. Flip motor direction for shooter launch rotation
        shooterP.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Set run mode to RUN_USING_ENCODER for closed-loop velocity control
        shooterP.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 4. Apply initial PIDF coefficients
        updatePIDF();

        telemetry.addLine("6000 RPM Shooter Test OpMode Initialized");
        telemetry.addLine("---------------------------------------");
        telemetry.addLine("[A] Start Motor  |  [B] Stop Motor  |  [X] Reset PIDF");
        telemetry.addLine("D-Pad Alone       : Target RPM Adjustment");
        telemetry.addLine("Hold Left Bumper  : Adjust P Gain");
        telemetry.addLine("Hold Right Bumper : Adjust F Gain");
    }

    /**
     * Helper method to update motor PIDF coefficients on the hardware controller.
     */
    private void updatePIDF() {
        if (shooterP != null) {
            shooterP.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
        }
    }

    @Override
    public void loop() {
        // --- 1. Gamepad Input & Edge Detection ---
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;
        boolean currentDpadRight = gamepad1.dpad_right;
        boolean currentDpadLeft = gamepad1.dpad_left;
        boolean currentX = gamepad1.x;

        boolean leftBumper = gamepad1.left_bumper;
        boolean rightBumper = gamepad1.right_bumper;

        // --- 2. Live Parameter Adjustment Logic ---
        if (leftBumper) {
            // Adjust P Gain (Left Bumper held)
            if (currentDpadUp && !previousDpadUp) p += 1.0;
            if (currentDpadDown && !previousDpadDown) p -= 1.0;
            if (currentDpadRight && !previousDpadRight) p += 0.1;
            if (currentDpadLeft && !previousDpadLeft) p -= 0.1;
            p = Math.max(0.0, p);
            updatePIDF();
        } else if (rightBumper) {
            // Adjust F Gain (Right Bumper held)
            if (currentDpadUp && !previousDpadUp) f += 0.5;
            if (currentDpadDown && !previousDpadDown) f -= 0.5;
            if (currentDpadRight && !previousDpadRight) f += 0.05;
            if (currentDpadLeft && !previousDpadLeft) f -= 0.05;
            f = Math.max(0.0, f);
            updatePIDF();
        } else {
            // Adjust Target RPM (No Bumpers held)
            if (currentDpadUp && !previousDpadUp) targetRpm += 100.0;
            if (currentDpadDown && !previousDpadDown) targetRpm -= 100.0;
            if (currentDpadRight && !previousDpadRight) targetRpm += 500.0;
            if (currentDpadLeft && !previousDpadLeft) targetRpm -= 500.0;
            targetRpm = Range.clip(targetRpm, 0.0, MAX_RPM);
        }

        // Reset PIDF coefficients to starting defaults
        if (currentX && !previousX) {
            p = DEFAULT_P;
            i = DEFAULT_I;
            d = DEFAULT_D;
            f = DEFAULT_F;
            updatePIDF();
        }

        // Update previous button states for edge detection
        previousDpadUp = currentDpadUp;
        previousDpadDown = currentDpadDown;
        previousDpadRight = currentDpadRight;
        previousDpadLeft = currentDpadLeft;
        previousX = currentX;

        // --- 3. Start / Stop Motor Controls ---
        if (gamepad1.a) {
            isRunning = true;
        } else if (gamepad1.b) {
            isRunning = false;
        }

        // --- 4. Motor Execution ---
        if (isRunning) {
            // Convert target RPM to ticks per second (RPM * 28 / 60)
            double targetTicksPerSec = targetRpm * (TICKS_PER_REV / 60.0);
            shooterP.setVelocity(targetTicksPerSec);
        } else {
            shooterP.setPower(0.0);
        }

        // --- 5. Driver Station Telemetry ---
        double actualTicksPerSec = shooterP.getVelocity();
        double actualRpm = actualTicksPerSec * (60.0 / TICKS_PER_REV);
        double rpmError = actualRpm - targetRpm;

        telemetry.addData("=== MOTOR STATE ===", isRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Target RPM", "%.0f RPM", targetRpm);
        telemetry.addData("Actual RPM", "%.1f RPM (Error: %+.1f RPM)", actualRpm, rpmError);
        telemetry.addData("Actual Velocity", "%.1f ticks/sec", actualTicksPerSec);
        telemetry.addData("Motor Power", "%.2f", shooterP.getPower());
        telemetry.addLine();

        telemetry.addData("=== ACTIVE PIDF ===", "P=%.2f  I=%.2f  D=%.2f  F=%.2f", p, i, d, f);
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS GUIDE ===");
        telemetry.addLine("[A] Start Motor  |  [B] Stop Motor  |  [X] Reset PIDF");
        telemetry.addLine("D-Pad Alone       : Up/Down ±100, Right/Left ±500 RPM");
        telemetry.addLine("Hold Left Bumper  : Up/Down ±1.0, Right/Left ±0.1 P");
        telemetry.addLine("Hold Right Bumper : Up/Down ±0.5, Right/Left ±0.05 F");
    }

    @Override
    public void stop() {
        if (shooterP != null) {
            shooterP.setPower(0.0);
        }
    }
}

