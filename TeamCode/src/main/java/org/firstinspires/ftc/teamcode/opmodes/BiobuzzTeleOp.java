package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrainMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PollenShooterMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.TransferMechanism;

/**
 * Biobuzz TeleOp OpMode
 *
 * Currently active:
 * - Mecanum Drivetrain
 * - Pollen Shooter Flywheel
 * - Transfer Motor
 * - Intake Motor
 * - Shooter Servo Gate
 *
 * Disabled / Commented out (Under Construction):
 * - Turret Aiming (no axon servo installed yet)
 * - Pinpoint Odometry (not ready)
 *
 * Controls:
 * - Left Stick Y: Drive forward/backward
 * - Left Stick X: Strafe left/right
 * - Right Stick X: Rotate
 * - A Button: Start shooting sequence (spins flywheel, opens gate & runs transfer motor when target speed reached)
 * - B Button: Stop shooting sequence (closes gate & stops transfer motor)
 * - X Button: Toggle flywheel on/off (default 2800 RPM)
 * - Y Button: Clear transfer & intake (reverses transfer & intake motors)
 * - Right Trigger: Run intake motor
 * - Right Bumper: Reverse intake motor
 * - [DISABLED] Left Trigger: Engage autoaim sequence
 * - [DISABLED] Left Bumper: Reset turret position back to zero position
 * - D-Pad Up: Increase shooter flywheel speed (100 RPM)
 * - D-Pad Down: Decrease shooter flywheel speed (100 RPM)
 * - [DISABLED] D-Pad Right: Step adjust turret towards the right
 * - [DISABLED] D-Pad Left: Step adjust turret towards the left
 */
@TeleOp(name = "Biobuzz TeleOp", group = "TeleOp")
public class BiobuzzTeleOp extends OpMode {

    // Reference mechanisms
    private final DriveTrainMechanism drivetrain = new DriveTrainMechanism();
    private final PollenShooterMechanism shooter = new PollenShooterMechanism();
    private final TransferMechanism transfer = new TransferMechanism();
    private final IntakeMechanism intake = new IntakeMechanism();

    // Mechanisms under construction (disabled for initial testing)
    // private final Autoaim autoaim = new Autoaim();
    // private final GoBildaPinpointDriver pinpointOdom; // Pinpoint Odometry (not ready)

    // State tracking variables
    private boolean isShootingSequenceActive = false;

    // Previous button states for edge detection / debouncing
    private boolean prevX = false;
    private boolean prevLeftBumper = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft = false;

    @Override
    public void init() {
        // Initialize ready mechanisms
        drivetrain.init(hardwareMap);
        transfer.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);

        // --- UNDER CONSTRUCTION: Disabled until hardware installed ---
        // autoaim.init(hardwareMap); // Turret / Axon servo not ready
        // pinpointOdom = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"); // Odometry not ready

        telemetry.addLine("Biobuzz TeleOp Initialized");
        telemetry.addLine("Active: Drivetrain, Shooter, Transfer, Intake, Shooter Gate");
        telemetry.addLine("Disabled: Turret, Pinpoint Odom");
    }

    @Override
    public void loop() {
        // --- 1. Mecanum Drive Controls ---
        double drive  = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double rotate =  gamepad1.right_stick_x;
        drivetrain.drive(drive, strafe, rotate);

        // --- 2. Button Edge Detection (X, D-pad Up/Down) ---
        boolean currentX = gamepad1.x;
        if (currentX && !prevX) {
            shooter.toggleFlywheel();
        }
        prevX = currentX;

        // --- DISABLED: Turret Reset (Left Bumper) - Axon Servo not installed ---
        /*
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !prevLeftBumper) {
            autoaim.resetPosition();
        }
        prevLeftBumper = currentLeftBumper;
        */

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

        // --- DISABLED: Turret Adjust (D-pad Right / Left) - Axon Servo not installed ---
        /*
        boolean currentDpadRight = gamepad1.dpad_right;
        if (currentDpadRight && !prevDpadRight) {
            autoaim.adjustRight();
        }
        prevDpadRight = currentDpadRight;

        boolean currentDpadLeft = gamepad1.dpad_left;
        if (currentDpadLeft && !prevDpadLeft) {
            autoaim.adjustLeft();
        }
        prevDpadLeft = currentDpadLeft;
        */

        // --- 3. Shooting Sequence Controls (A / B Buttons) ---
        if (gamepad1.a) {
            isShootingSequenceActive = true;
        } else if (gamepad1.b) {
            isShootingSequenceActive = false;
        }

        // --- 4. Pollen Transfer & Shooter Execution ---
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
            // Shooting stopped - close gate and stop transfer unless Y (clear) is active
            transfer.closeShooterGate();
            if (gamepad1.y) {
                transfer.reverseTransfer();
            } else {
                transfer.stopTransfer();
            }

            // Run flywheel if toggled ON manually via X button, otherwise stop
            if (shooter.isFlywheelOn()) {
                shooter.runShooter();
            } else {
                shooter.stopShooter();
            }
        }

        // --- 5. Intake Controls (Y Button, Right Trigger, Right Bumper) ---
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

        // --- 6. Autoaim & Turret Controls --- DISABLED: Axon Servo not installed
        /*
        boolean engageAutoaim = gamepad1.left_trigger > 0.1;
        autoaim.update(engageAutoaim);
        */

        // --- 7. Pinpoint Odometry Update --- DISABLED: Odometry not ready
        /*
        pinpointOdom.update();
        */

        // --- 8. Telemetry ---
        telemetry.addData("--- SHOOTER (READY) ---", "");
        telemetry.addData("Shooting Sequence", isShootingSequenceActive ? "ACTIVE" : "STOPPED");
        telemetry.addData("Flywheel Toggle", shooter.isFlywheelOn() ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f RPM", shooter.getTargetRpm());
        telemetry.addData("Actual RPM", "%.0f RPM", shooter.getCurrentRpm());
        telemetry.addData("Actual Velocity", "%.1f ticks/sec", shooter.getVelocityTicksPerSec());
        telemetry.addData("Encoder Pos", shooter.getEncoderPosition());
        telemetry.addData("Speed Permissive", shooter.isAtTargetSpeed() ? "READY" : "WAITING");

        telemetry.addData("--- TRANSFER (READY) ---", "");
        telemetry.addData("Transfer Power", "%.2f", transfer.getTransferPower());
        telemetry.addData("Shooter Gate", transfer.isShooterGateOpen() ? "OPEN (0.8)" : "CLOSED (1.0)");

        telemetry.addData("--- INTAKE (READY) ---", "");
        telemetry.addData("Intake Power", "%.2f", intake.getIntakePower());

        telemetry.addData("--- TURRET / AUTOAIM ---", "DISABLED (No Axon Servo)");
        telemetry.addData("--- PINPOINT ODOMETRY ---", "DISABLED (Under Construction)");

        telemetry.addData("--- DRIVETRAIN (READY) ---", "");
        telemetry.addData("Drive Inputs", "D: %.2f, S: %.2f, R: %.2f", drive, strafe, rotate);
    }

    @Override
    public void stop() {
        drivetrain.setMotorPowers(0, 0, 0, 0);
        intake.stopIntake();
        transfer.stopTransfer();
        transfer.closeShooterGate();
        shooter.stopShooter();
    }
}
