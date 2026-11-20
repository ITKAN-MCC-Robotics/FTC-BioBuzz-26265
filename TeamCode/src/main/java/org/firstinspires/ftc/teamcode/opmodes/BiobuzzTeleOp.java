package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Autoaim;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrainMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PollenShooterMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.TransferMechanism;

/**
 * Biobuzz TeleOp OpMode
 *
 * Active Mechanisms:
 * - Mecanum Drivetrain
 * - Pollen Shooter Flywheel
 * - Transfer Motor & Shooter Servo Gate
 * - Intake Motor
 * - GoBilda Pinpoint Odometry Computer (Field Localization)
 * - Turret Autoaim (Axon Mini MK2 Servo "turretP" & Encoder "turretPfeedback")
 *
 * Controls:
 * - Left Stick Y: Drive forward/backward
 * - Left Stick X: Strafe left/right
 * - Right Stick X: Rotate
 * - Left Trigger: Hold to engage Turret Autoaim
 * - Left Bumper: Force turret back to center position (0.5)
 * - A Button: Start shooting sequence (spins flywheel, opens gate & runs transfer motor when target speed reached)
 * - B Button: Stop shooting sequence (closes gate & stops transfer motor)
 * - X Button: Toggle flywheel on/off (default 3000 RPM)
 * - Y Button: Clear transfer & intake (reverses transfer & intake motors, opens gate)
 * - Right Trigger: Run intake motor & transfer motor (opens gate to feed pollen into staging/shooter)
 * - Right Bumper: Reverse intake motor only (transfer stopped, gate closed)
 * - Back Button: Reset Pinpoint Odometry position & IMU to alliance starting pose
 * - D-Pad Up: Increase shooter flywheel speed (100 RPM)
 * - D-Pad Down: Decrease shooter flywheel speed (100 RPM)
 * - D-Pad Right: Toggle Flower Ramp servo position (Open 0.435 / Closed 0.75)
 * - D-Pad Left: Toggle Shooter Gate servo position (Open 0.53 / Closed 0.66)
 */
@TeleOp(name = "Biobuzz TeleOp", group = "TeleOp")
public class BiobuzzTeleOp extends OpMode {

    /**
     * Alliance selection enum for starting position and field targeting.
     */
    public enum Alliance {
        RED,
        BLUE,
        UNSELECTED
    }

    // Reference mechanisms
    private final DriveTrainMechanism drivetrain = new DriveTrainMechanism();
    private final PollenShooterMechanism shooter = new PollenShooterMechanism();
    private final TransferMechanism transfer = new TransferMechanism();
    private final IntakeMechanism intake = new IntakeMechanism();
    private final PinpointMechanism pinpoint = new PinpointMechanism();
    private final Autoaim autoaim = new Autoaim();

    // State tracking variables
    private Alliance selectedAlliance = Alliance.UNSELECTED;
    private boolean isShootingSequenceActive = false;
    private boolean isManualGateOpen = false;

    // Previous button states for edge detection / debouncing
    private boolean prevX = false;
    private boolean prevBack = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft = false;

    @Override
    public void init() {
        // Initialize active mechanisms
        drivetrain.init(hardwareMap);
        transfer.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        pinpoint.init(hardwareMap);
        autoaim.init(hardwareMap);

        telemetry.addLine("=== BIOBUZZ TELEOP INITIALIZED ===");
        telemetry.addLine("Active: Drivetrain, Shooter, Transfer, Intake, Shooter Gate, Flower Ramp, Pinpoint Odom, Turret Autoaim");
        telemetry.addLine("");
        telemetry.addLine("=== ALLIANCE SELECTION REQUIRED ===");
        telemetry.addLine("Press [B] for RED Alliance  (Pedro Start: 85, 132, -90°)");
        telemetry.addLine("Press [X] for BLUE Alliance (Pedro Start: 54.5, 8.5, 90°)");
    }

    @Override
    public void init_loop() {
        if (gamepad1.b) {
            selectedAlliance = Alliance.RED;
            pinpoint.resetToAudienceWallRed();
        } else if (gamepad1.x) {
            selectedAlliance = Alliance.BLUE;
            pinpoint.resetToAudienceWallBlue();
        }

        telemetry.addLine("=== ALLIANCE SELECTION ===");
        telemetry.addLine("Press [B] for RED Alliance | Press [X] for BLUE Alliance");
        telemetry.addData("Selected Alliance", selectedAlliance == Alliance.UNSELECTED ? "NONE (Default: RED)" : selectedAlliance);
        telemetry.addData("Pedro Start Pose", "X: %.1f\", Y: %.1f\", Heading: %.1f°",
                pinpoint.getPedroX(), pinpoint.getPedroY(), pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start() {
        // Fallback: If no alliance selected during init_loop, default to RED alliance
        if (selectedAlliance == Alliance.UNSELECTED) {
            selectedAlliance = Alliance.RED;
            pinpoint.resetToAudienceWallRed();
        }
    }

    @Override
    public void loop() {
        // --- 1. Update Pinpoint Odometry ---
        pinpoint.update();

        // Reset Pinpoint Position & IMU to selected alliance starting position if Back button pressed
        boolean currentBack = gamepad1.back;
        if (currentBack && !prevBack) {
            if (selectedAlliance == Alliance.BLUE) {
                pinpoint.resetToAudienceWallBlue();
            } else {
                pinpoint.resetToAudienceWallRed();
            }
        }
        prevBack = currentBack;

        // --- 2. Mecanum Drive Controls ---
        double drive  = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double rotate =  gamepad1.right_stick_x;
        drivetrain.drive(drive, strafe, rotate);

        // --- 3. Button Edge Detection (X, D-pad Up/Down/Left/Right) ---
        boolean currentX = gamepad1.x;
        if (currentX && !prevX) {
            shooter.toggleFlywheel();
        }
        prevX = currentX;

        boolean currentDpadUp = gamepad1.dpad_up;
        if (currentDpadUp && !prevDpadUp) {
            shooter.adjustRpm(100.0);
        }
        prevDpadUp = currentDpadUp;

        boolean currentDpadDown = gamepad1.dpad_down;
        if (currentDpadDown && !prevDpadDown) {
            shooter.adjustRpm(-100.0);
        }
        prevDpadDown = currentDpadDown;

        boolean currentDpadRight = gamepad1.dpad_right;
        if (currentDpadRight && !prevDpadRight) {
            transfer.toggleFlowerRamp();
        }
        prevDpadRight = currentDpadRight;

        boolean currentDpadLeft = gamepad1.dpad_left;
        if (currentDpadLeft && !prevDpadLeft) {
            isManualGateOpen = !isManualGateOpen;
        }
        prevDpadLeft = currentDpadLeft;

        // --- 4. Shooting Sequence Controls (A / B Buttons) ---
        if (gamepad1.a) {
            if (!isShootingSequenceActive) {
                // Clear any transfer stall state when starting shooting sequence
                transfer.resetStall();
            }
            isShootingSequenceActive = true;
            isManualGateOpen = false;
        } else if (gamepad1.b) {
            isShootingSequenceActive = false;
            isManualGateOpen = false;
        }

        // --- 5. Pollen Transfer & Shooter Execution ---
        if (isShootingSequenceActive) {
            shooter.runShooter();
            // Permissive check: run transfer and open gate only once target speed is reached
            if (shooter.isAtTargetSpeed()) {
                transfer.openShooterGate();
                transfer.runTransfer();
            } else {
                transfer.closeShooterGate();
                transfer.stopTransfer();
            }
        } else {
            if (gamepad1.y) {
                // Y button: clear transfer & intake jam by reversing transfer motor and opening gate
                transfer.resetStall();
                transfer.openShooterGate();
                transfer.reverseTransfer();
            } else if (gamepad1.right_trigger > 0.1) {
                // Right Trigger (Intake): run transfer forward and keep gate closed (unless manually toggled open via D-pad Left)
                if (isManualGateOpen) {
                    transfer.openShooterGate();
                } else {
                    transfer.closeShooterGate();
                }
                transfer.runTransfer();
            } else {
                // Right Bumper (Reverse Intake) or Idle: stop transfer motor
                transfer.stopTransfer();
                // D-pad manual gate control: open gate if manual toggle (D-pad Left) is active, otherwise close
                if (isManualGateOpen) {
                    transfer.openShooterGate();
                } else {
                    transfer.closeShooterGate();
                }
            }

            // Run flywheel if toggled ON manually via X button, otherwise stop
            if (shooter.isFlywheelOn()) {
                shooter.runShooter();
            } else {
                shooter.stopShooter();
            }
        }

        // --- 6. Intake Controls (Y Button, Right Trigger, Right Bumper) ---
        if (gamepad1.y) {
            // Y button: clear transfer and intake by reversing both motors
            intake.reverseIntake();
        } else if (gamepad1.right_trigger > 0.1) {
            intake.setIntakePower(gamepad1.right_trigger);
        } else if (gamepad1.right_bumper) {
            intake.reverseIntake();
        } else {
            intake.stopIntake();
        }

        // --- 7. Turret Autoaim Execution ---
        // Left Trigger engages auto-aim; Left Bumper manually forces center position (0.5)
        boolean engageAutoaim = gamepad1.left_trigger > 0.1;
        boolean forceCenter = gamepad1.left_bumper;

        double robotPedroX = pinpoint.getPedroX();
        double robotPedroY = pinpoint.getPedroY();
        double robotPedroHeadingRad = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

        autoaim.update(engageAutoaim, forceCenter, robotPedroX, robotPedroY, robotPedroHeadingRad, selectedAlliance == Alliance.BLUE);

        // --- Update Transfer Stall Detection ---
        transfer.update();

        // --- 8. Telemetry ---
        telemetry.addData("--- ALLIANCE ---", selectedAlliance);
        telemetry.addData("--- PINPOINT ODOMETRY ---", pinpoint.isInitialized() ? "OK" : "ERROR");
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pedro Pos (X, Y in)", "X: %.2f, Y: %.2f", robotPedroX, robotPedroY);
        telemetry.addData("Heading (Deg)", "%.1f°", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("FTC Pos (X, Y in)", "X: %.2f, Y: %.2f", pinpoint.getX(DistanceUnit.INCH), pinpoint.getY(DistanceUnit.INCH));

        telemetry.addData("--- TURRET AUTOAIM ---", forceCenter ? "FORCE CENTER (LB)" : (autoaim.isAutoaimActive() ? "AUTO-AIM ENGAGED (LT)" : "IDLE CENTER"));
        telemetry.addData("Active Goal Target (X, Y)", "X: %.2f, Y: %.2f", autoaim.getActiveTargetX(), autoaim.getActiveTargetY());
        telemetry.addData("Turret Servo Pos", "%.3f %s", autoaim.getTurretPosition(), autoaim.isServoAttached() ? "(SERVO OK)" : "(SERVO MISSING)");
        telemetry.addData("Target Angle (Calc)", "%.1f°", autoaim.getCalculatedTargetAngleDeg());
        telemetry.addData("Turret Angle (Clamped)", "%.1f° %s", autoaim.getClampedTurretAngleDeg(), autoaim.isClamped() ? "[LIMIT " + Autoaim.MAX_TURRET_LIMIT_DEG + "°]" : "");
        telemetry.addData("Feedback Encoder", "%s | Volts: %.2f V | Angle: %.1f°",
                autoaim.isEncoderAttached() ? "OK" : "MISSING", autoaim.getFeedbackVoltage(), autoaim.getFeedbackAngleDeg());

        telemetry.addData("--- SHOOTER (READY) ---", "");
        telemetry.addData("Shooting Sequence", isShootingSequenceActive ? "ACTIVE" : "STOPPED");
        telemetry.addData("Flywheel Toggle", shooter.isFlywheelOn() ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f RPM", shooter.getTargetRpm());
        telemetry.addData("Actual RPM", "%.0f RPM", shooter.getCurrentRpm());
        telemetry.addData("Speed Permissive", shooter.isAtTargetSpeed() ? "READY" : "WAITING");

        telemetry.addData("--- TRANSFER ---", transfer.isServoAttached() ? "SERVO READY" : "SERVO UNCONFIGURED / NULL");
        telemetry.addData("Transfer Power", "%.2f", transfer.getTransferPower());
        telemetry.addData("Transfer Current", "%.2f A", transfer.getMotorCurrent());
        telemetry.addData("Transfer Stall State", transfer.isStalled() ? "STALLED" : "NORMAL");
        telemetry.addData("Shooter Gate State", transfer.isShooterGateOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Gate Servo Position", "%.3f", transfer.getShooterGatePosition());

        telemetry.addData("--- FLOWER RAMP ---", transfer.isFlowerRampAttached() ? "SERVO READY" : "SERVO UNCONFIGURED / NULL");
        telemetry.addData("Flower Ramp State", transfer.isFlowerRampOpen() ? "OPEN (0.435)" : "CLOSED (0.75)");
        telemetry.addData("Flower Ramp Position", "%.3f", transfer.getFlowerRampPosition());

        telemetry.addData("--- INTAKE (READY) ---", "");
        telemetry.addData("Intake Power", "%.2f", intake.getIntakePower());

        telemetry.addData("--- DRIVETRAIN (READY) ---", "");
        telemetry.addData("Drive Inputs", "D: %.2f, S: %.2f, R: %.2f", drive, strafe, rotate);
    }

    @Override
    public void stop() {
        drivetrain.setMotorPowers(0, 0, 0, 0);
        intake.stopIntake();
        transfer.stopTransfer();
        transfer.closeShooterGate();
        transfer.closeFlowerRamp();
        shooter.stopShooter();
    }
}
