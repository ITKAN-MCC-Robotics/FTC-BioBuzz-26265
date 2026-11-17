package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Autoaim;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrainMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PollenShooterMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.TransferMechanism;

/**
 * ShooterTuningTeleOp - TeleOp OpMode for testing and live dynamic tuning
 * of the Pollen Shooter P and F factors, based on BiobuzzTeleOp.
 *
 * Tuning Controls:
 * - Left Trigger     : Switch active tuning factor (P Factor <-> F Factor)
 * - D-Pad Right      : Increase active factor value by increment step
 * - D-Pad Left       : Decrease active factor value by increment step
 * - Left Bumper      : Cycle increment step size (P: 0.01-10.0 | F: 1000, 100, 10)
 * - D-Pad Up         : Increase target flywheel speed (100 RPM)
 * - D-Pad Down       : Decrease target flywheel speed (100 RPM)
 *
 * TeleOp Controls (Same as BiobuzzTeleOp):
 * - Left Stick Y     : Drive forward/backward
 * - Left Stick X     : Strafe left/right
 * - Right Stick X    : Rotate
 * - A Button         : Start shooting sequence
 * - B Button         : Stop shooting sequence
 * - X Button         : Toggle flywheel on/off
 * - Y Button         : Clear transfer & intake
 * - Right Trigger    : Run intake motor & transfer motor
 * - Right Bumper     : Reverse intake motor
 * - Back Button      : Reset Pinpoint Odometry position to alliance start
 */
@TeleOp(name = "Shooter Tuning TeleOp", group = "TeleOp")
public class ShooterTuningTeleOp extends OpMode {

    public enum Alliance {
        RED,
        BLUE,
        UNSELECTED
    }

    public enum TuningFactor {
        P_FACTOR,
        F_FACTOR
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

    // Tuning state variables
    private TuningFactor activeFactor = TuningFactor.P_FACTOR;
    private static final double[] P_INCREMENT_SIZES = {0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0};
    private static final double[] F_INCREMENT_SIZES = {10.0, 1.0, 0.1};

    private int pIncrementIndex = 2; // Default 0.10 for P Factor
    private int fIncrementIndex = 1; // Default 100 for F Factor

    // Previous button states for edge detection / debouncing
    private boolean prevX = false;
    private boolean prevBack = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft = false;
    private boolean prevLeftBumper = false;
    private boolean prevLeftTrigger = false;

    @Override
    public void init() {
        // Initialize active mechanisms
        drivetrain.init(hardwareMap);
        transfer.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        pinpoint.init(hardwareMap);
        autoaim.init(hardwareMap);

        telemetry.addLine("=== SHOOTER TUNING TELEOP INITIALIZED ===");
        telemetry.addLine("Active: Drivetrain, Shooter, Transfer, Intake, Shooter Gate, Flower Ramp, Pinpoint Odom, Turret Autoaim");
        telemetry.addLine("");
        telemetry.addLine("=== ALLIANCE SELECTION REQUIRED ===");
        telemetry.addLine("Press [B] for RED Alliance  (Pedro Start: 36, 9, 90°)");
        telemetry.addLine("Press [X] for BLUE Alliance (Pedro Start: 106, 132, 270°)");
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

        // --- 3. PIDF Tuning Controls ---
        // Left Trigger: Switch between P Factor and F Factor
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.3;
        if (currentLeftTrigger && !prevLeftTrigger) {
            if (activeFactor == TuningFactor.P_FACTOR) {
                activeFactor = TuningFactor.F_FACTOR;
            } else {
                activeFactor = TuningFactor.P_FACTOR;
            }
        }
        prevLeftTrigger = currentLeftTrigger;

        // Left Bumper: Change increment size (P: 0.01-10.0 | F: 1000, 100, 10)
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !prevLeftBumper) {
            if (activeFactor == TuningFactor.P_FACTOR) {
                pIncrementIndex = (pIncrementIndex + 1) % P_INCREMENT_SIZES.length;
            } else {
                fIncrementIndex = (fIncrementIndex + 1) % F_INCREMENT_SIZES.length;
            }
        }
        prevLeftBumper = currentLeftBumper;

        double stepSize = (activeFactor == TuningFactor.P_FACTOR)
                ? P_INCREMENT_SIZES[pIncrementIndex]
                : F_INCREMENT_SIZES[fIncrementIndex];

        // D-Pad Left / Right: Increase / Decrease active factor value
        boolean currentDpadRight = gamepad1.dpad_right;
        if (currentDpadRight && !prevDpadRight) {
            if (activeFactor == TuningFactor.P_FACTOR) {
                shooter.setP(shooter.getP() + stepSize);
            } else {
                shooter.setF(shooter.getF() + stepSize);
            }
        }
        prevDpadRight = currentDpadRight;

        boolean currentDpadLeft = gamepad1.dpad_left;
        if (currentDpadLeft && !prevDpadLeft) {
            if (activeFactor == TuningFactor.P_FACTOR) {
                shooter.setP(Math.max(0.0, shooter.getP() - stepSize));
            } else {
                shooter.setF(Math.max(0.0, shooter.getF() - stepSize));
            }
        }
        prevDpadLeft = currentDpadLeft;

        // D-Pad Up / Down: Adjust target shooter RPM
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

        // X Button: Toggle Flywheel ON/OFF
        boolean currentX = gamepad1.x;
        if (currentX && !prevX) {
            shooter.toggleFlywheel();
        }
        prevX = currentX;

        // --- 4. Shooting Sequence Controls (A / B Buttons) ---
        if (gamepad1.a) {
            if (!isShootingSequenceActive) {
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
            if (shooter.isAtTargetSpeed()) {
                transfer.openShooterGate();
                transfer.runTransfer();
            } else {
                transfer.closeShooterGate();
                transfer.stopTransfer();
            }
        } else {
            if (gamepad1.y) {
                transfer.resetStall();
                transfer.openShooterGate();
                transfer.reverseTransfer();
            } else if (gamepad1.right_trigger > 0.1) {
                if (isManualGateOpen) {
                    transfer.openShooterGate();
                } else {
                    transfer.closeShooterGate();
                }
                transfer.runTransfer();
            } else {
                transfer.stopTransfer();
                if (isManualGateOpen) {
                    transfer.openShooterGate();
                } else {
                    transfer.closeShooterGate();
                }
            }

            if (shooter.isFlywheelOn()) {
                shooter.runShooter();
            } else {
                shooter.stopShooter();
            }
        }

        // --- 6. Intake Controls (Y Button, Right Trigger, Right Bumper) ---
        if (gamepad1.y) {
            intake.reverseIntake();
        } else if (gamepad1.right_trigger > 0.1) {
            intake.setIntakePower(gamepad1.right_trigger);
        } else if (gamepad1.right_bumper) {
            intake.reverseIntake();
        } else {
            intake.stopIntake();
        }

        // --- 7. Turret Autoaim Execution ---
        double robotPedroX = pinpoint.getPedroX();
        double robotPedroY = pinpoint.getPedroY();
        double robotPedroHeadingRad = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

        autoaim.update(false, false, robotPedroX, robotPedroY, robotPedroHeadingRad, selectedAlliance == Alliance.BLUE);

        // --- Update Transfer Stall Detection ---
        transfer.update();

        // --- 8. Telemetry Focused on Shooter Speed, P Factor, and F Factor ---
        telemetry.addLine("=========================================");
        telemetry.addLine("      SHOOTER PIDF TUNING TELEOP        ");
        telemetry.addLine("=========================================");
        telemetry.addData("ACTIVE TUNING FACTOR", activeFactor == TuningFactor.P_FACTOR ? ">>> P FACTOR <<<" : ">>> F FACTOR <<<");
        if (activeFactor == TuningFactor.P_FACTOR) {
            telemetry.addData("INCREMENT STEP SIZE ", "%.2f (Press LB to change)", stepSize);
        } else {
            telemetry.addData("INCREMENT STEP SIZE ", "%.0f (Press LB to change)", stepSize);
        }
        telemetry.addLine("-----------------------------------------");
        telemetry.addData("P Factor (Proportional)", "%.4f", shooter.getP());
        telemetry.addData("F Factor (Feedforward) ", "%.4f", shooter.getF());
        telemetry.addData("I / D Factors          ", "I: %.2f | D: %.2f", shooter.getI(), shooter.getD());
        telemetry.addLine("-----------------------------------------");
        telemetry.addData("Target Speed (RPM)     ", "%.0f RPM", shooter.getTargetRpm());
        telemetry.addData("Actual Speed (RPM)     ", "%.1f RPM", shooter.getCurrentRpm());
        telemetry.addData("Speed Error (RPM)      ", "%+.1f RPM", shooter.getCurrentRpm() - shooter.getTargetRpm());
        telemetry.addData("Velocity Ticks/Sec     ", "%.1f ticks/s", shooter.getVelocityTicksPerSec());
        telemetry.addData("Flywheel Power State   ", shooter.isFlywheelOn() ? "ENABLED (RUNNING)" : "DISABLED (STOPPED)");
        telemetry.addData("Speed Permissive       ", shooter.isAtTargetSpeed() ? "READY (At Target Speed)" : "SPINNING UP");
        telemetry.addLine("=========================================");
        telemetry.addLine("--- TUNING CONTROLS GUIDE ---");
        telemetry.addLine("Left Trigger     : Switch active factor (P <-> F)");
        telemetry.addLine("D-Pad Left/Right : Decrease / Increase active factor");
        telemetry.addLine("Left Bumper      : Cycle increment step size");
        telemetry.addLine("D-Pad Up/Down    : Adjust Target RPM (±100)");
        telemetry.addLine("X Button         : Toggle Flywheel ON/OFF");
        telemetry.addLine("A / B Buttons    : Start / Stop Shooting Sequence");
        telemetry.addLine("=========================================");
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Pinpoint Pose", "X: %.1f\", Y: %.1f\", Heading: %.1f°",
                robotPedroX, robotPedroY, pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Shooter Gate", transfer.isShooterGateOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Flower Ramp", transfer.isFlowerRampOpen() ? "OPEN" : "CLOSED");
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
