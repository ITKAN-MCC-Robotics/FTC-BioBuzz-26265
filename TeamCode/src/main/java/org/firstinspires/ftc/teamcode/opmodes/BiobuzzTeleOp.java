package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
 *
 * Disabled / Under Construction:
 * - Turret Aiming (no axon servo installed yet)
 *
 * Controls:
 * - Left Stick Y: Drive forward/backward
 * - Left Stick X: Strafe left/right
 * - Right Stick X: Rotate
 * - A Button: Start shooting sequence (spins flywheel, opens gate & runs transfer motor when target speed reached)
 * - B Button: Stop shooting sequence (closes gate & stops transfer motor)
 * - X Button: Toggle flywheel on/off (default 2800 RPM)
 * - Y Button: Clear transfer & intake (reverses transfer & intake motors)
 * - Right Trigger: Run intake motor & transfer motor (gate remains closed)
 * - Right Bumper: Reverse intake motor only (transfer stopped, gate closed)
 * - Back Button: Reset Pinpoint Odometry position & IMU to (0,0,0)
 * - D-Pad Up: Increase shooter flywheel speed (100 RPM)
 * - D-Pad Down: Decrease shooter flywheel speed (100 RPM)
 */
@TeleOp(name = "Biobuzz TeleOp", group = "TeleOp")
public class BiobuzzTeleOp extends OpMode {

    // Reference mechanisms
    private final DriveTrainMechanism drivetrain = new DriveTrainMechanism();
    private final PollenShooterMechanism shooter = new PollenShooterMechanism();
    private final TransferMechanism transfer = new TransferMechanism();
    private final IntakeMechanism intake = new IntakeMechanism();
    private final PinpointMechanism pinpoint = new PinpointMechanism();

    // Mechanisms under construction
    // private final Autoaim autoaim = new Autoaim();

    // State tracking variables
    private boolean isShootingSequenceActive = false;

    // Previous button states for edge detection / debouncing
    private boolean prevX = false;
    private boolean prevBack = false;
    private boolean prevLeftBumper = false;
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

        // --- UNDER CONSTRUCTION: Disabled until hardware installed ---
        // autoaim.init(hardwareMap); // Turret / Axon servo not ready

        telemetry.addLine("Biobuzz TeleOp Initialized");
        telemetry.addLine("Active: Drivetrain, Shooter, Transfer, Intake, Shooter Gate, Pinpoint Odom");
        telemetry.addLine("Disabled: Turret Autoaim");
    }

    @Override
    public void loop() {
        // --- 1. Update Pinpoint Odometry ---
        pinpoint.update();

        // Reset Pinpoint Position & IMU if Back button pressed while stationary
        boolean currentBack = gamepad1.back;
        if (currentBack && !prevBack) {
            pinpoint.resetPosAndIMU();
        }
        prevBack = currentBack;

        // --- 2. Mecanum Drive Controls ---
        double drive  = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double rotate =  gamepad1.right_stick_x;
        drivetrain.drive(drive, strafe, rotate);

        // --- 3. Button Edge Detection (X, D-pad Up/Down) ---
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

        // --- 4. Shooting Sequence Controls (A / B Buttons) ---
        if (gamepad1.a) {
            isShootingSequenceActive = true;
        } else if (gamepad1.b) {
            isShootingSequenceActive = false;
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
            // Shooting stopped - keep gate closed
            transfer.closeShooterGate();

            if (gamepad1.y) {
                // Y button: clear transfer & intake jam by reversing transfer motor
                transfer.reverseTransfer();
            } else if (gamepad1.right_trigger > 0.1) {
                // Right Trigger (Intake): run transfer forward to feed elements into staging area
                transfer.runTransfer();
            } else {
                // Right Bumper (Reverse Intake) or Idle: stop transfer motor to keep staged elements inside
                transfer.stopTransfer();
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

        // --- 7. Telemetry ---
        telemetry.addData("--- PINPOINT ODOMETRY ---", pinpoint.isInitialized() ? "OK" : "ERROR");
        telemetry.addData("Status", pinpoint.getDeviceStatus());
        telemetry.addData("FTC Pos (X, Y in)", "X: %.2f, Y: %.2f", pinpoint.getX(DistanceUnit.INCH), pinpoint.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (Deg)", "%.1f°", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pedro Pos (X, Y)", "X: %.2f, Y: %.2f", pinpoint.getPedroX(), pinpoint.getPedroY());
        telemetry.addData("Target Angle (Red)", "%.1f°", Math.toDegrees(pinpoint.getAngleToRedTarget()));
        telemetry.addData("Target Angle (Blue)", "%.1f°", Math.toDegrees(pinpoint.getAngleToBlueTarget()));

        telemetry.addData("--- SHOOTER (READY) ---", "");
        telemetry.addData("Shooting Sequence", isShootingSequenceActive ? "ACTIVE" : "STOPPED");
        telemetry.addData("Flywheel Toggle", shooter.isFlywheelOn() ? "ON" : "OFF");
        telemetry.addData("Target RPM", "%.0f RPM", shooter.getTargetRpm());
        telemetry.addData("Actual RPM", "%.0f RPM", shooter.getCurrentRpm());
        telemetry.addData("Speed Permissive", shooter.isAtTargetSpeed() ? "READY" : "WAITING");

        telemetry.addData("--- TRANSFER (READY) ---", "");
        telemetry.addData("Transfer Power", "%.2f", transfer.getTransferPower());
        telemetry.addData("Shooter Gate", transfer.isShooterGateOpen() ? "OPEN (0.8)" : "CLOSED (1.0)");

        telemetry.addData("--- INTAKE (READY) ---", "");
        telemetry.addData("Intake Power", "%.2f", intake.getIntakePower());

        telemetry.addData("--- TURRET / AUTOAIM ---", "DISABLED (No Axon Servo)");

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
