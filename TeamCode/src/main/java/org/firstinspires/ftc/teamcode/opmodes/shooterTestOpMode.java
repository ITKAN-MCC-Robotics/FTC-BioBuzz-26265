package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

/**
 * shooterTestOpMode - TeleOp OpMode for testing a 6000 RPM shooter motor.
 * Controls:
 * - Gamepad A         : Start motor at target RPM
 * - Gamepad B         : Stop motor
 * - D-Pad Up / Down   : +/- 100 RPM
 * - D-Pad Right / Left: +/- 500 RPM
 */
@TeleOp(name = "shooterTestOpMode", group = "Testing")
public class shooterTestOpMode extends OpMode {

    private DcMotorEx shooter;

    // 6000 RPM Motor Specs (28 ticks per revolution for goBILDA/REV 1:1 bare motor)
    private static final double MAX_RPM = 6000.0;
    private static final double TICKS_PER_REV = 28.0;

    private double targetRpm = 3000.0; // Default starting target RPM
    private boolean isRunning = false;

    // Previous D-pad button states for edge detection
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;
    private boolean previousDpadRight = false;
    private boolean previousDpadLeft = false;

    @Override
    public void init() {
        // 1. Hardware Map lookup for motor "shooter"
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // 2. Flip motor direction for shooter launch rotation
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // 3. Set run mode to RUN_USING_ENCODER for closed-loop velocity control
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // 4. Configure optimal PIDF coefficients for 6000 RPM motor on REV Hub
        // Max Ticks/Sec = (6000 RPM * 28 TPR) / 60 = 2800 ticks/sec
        double maxTicksPerSecond = (MAX_RPM * TICKS_PER_REV) / 60.0;
        double f = 32767.0 / maxTicksPerSecond; // Feedforward F ≈ 11.7
        double p = 10.0;
        double i = 1.0;
        double d = 0.0;

        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));

        telemetry.addLine("6000 RPM Shooter Test OpMode Initialized");
        telemetry.addLine("---------------------------------------");
        telemetry.addLine("Gamepad A         : Start Motor");
        telemetry.addLine("Gamepad B         : Stop Motor");
        telemetry.addLine("D-Pad Up/Down     : +/- 100 RPM");
        telemetry.addLine("D-Pad Right/Left  : +/- 500 RPM");
    }

    @Override
    public void loop() {
        // --- 1. D-Pad RPM Adjustments (Edge Detection) ---
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;
        boolean currentDpadRight = gamepad1.dpad_right;
        boolean currentDpadLeft = gamepad1.dpad_left;

        if (currentDpadUp && !previousDpadUp) {
            targetRpm += 100.0;
        }
        if (currentDpadDown && !previousDpadDown) {
            targetRpm -= 100.0;
        }
        if (currentDpadRight && !previousDpadRight) {
            targetRpm += 500.0;
        }
        if (currentDpadLeft && !previousDpadLeft) {
            targetRpm -= 500.0;
        }

        previousDpadUp = currentDpadUp;
        previousDpadDown = currentDpadDown;
        previousDpadRight = currentDpadRight;
        previousDpadLeft = currentDpadLeft;

        // Clamp target RPM between 0 and 6000 RPM
        targetRpm = Range.clip(targetRpm, 0.0, MAX_RPM);

        // --- 2. Start / Stop Motor Controls (Gamepad A & B) ---
        if (gamepad1.a) {
            isRunning = true;
        } else if (gamepad1.b) {
            isRunning = false;
        }

        // --- 3. Motor Execution ---
        if (isRunning) {
            // Convert target RPM to ticks per second (RPM * 28 / 60)
            double targetTicksPerSec = targetRpm * (TICKS_PER_REV / 60.0);
            shooter.setVelocity(targetTicksPerSec);
        } else {
            shooter.setPower(0.0);
        }

        // --- 4. Driver Station Telemetry ---
        double actualTicksPerSec = shooter.getVelocity();
        double actualRpm = actualTicksPerSec * (60.0 / TICKS_PER_REV);

        PIDFCoefficients pidf = shooter.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Motor State", isRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Motor Direction", "REVERSE");
        telemetry.addData("Motor Type", "6000 RPM (28 TPR)");
        telemetry.addData("Target RPM", "%.0f RPM", targetRpm);
        telemetry.addData("Actual RPM", "%.1f RPM", actualRpm);
        telemetry.addData("Actual Ticks/Sec", "%.1f", actualTicksPerSec);
        telemetry.addData("PIDF Values", "P=%.1f I=%.1f D=%.1f F=%.2f", pidf.p, pidf.i, pidf.d, pidf.f);
        telemetry.addData("Encoder Position", shooter.getCurrentPosition());
        telemetry.addData("Motor Power", "%.2f", shooter.getPower());
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.setPower(0.0);
        }
    }
}
