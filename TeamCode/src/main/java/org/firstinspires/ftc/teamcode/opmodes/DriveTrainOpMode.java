package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrainMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PollenShooterMechanism;

/**
 * DriveTrainOpMode
 *
 * TeleOp mode for testing the Mecanum drivetrain.
 * Uses Gamepad 1 for driving:
 * - Left Stick Y: Forward/Backward
 * - Left Stick X: Strafe Left/Right
 * - Right Stick X: Rotate Clockwise/Counter-clockwise
 * - Right/Left Trigger: Intake Motor
 * - Y Button: Toggle Servo Flipper (0° to 90°)
 */
@TeleOp(name = "Mecanum Drive Test", group = "Testing")
public class DriveTrainOpMode extends OpMode {
    // Instantiate the drivetrain mechanism
    private DriveTrainMechanism drivetrain = new DriveTrainMechanism();
    private PollenShooterMechanism pollenShooter = new PollenShooterMechanism();
    private DcMotor intake;
    private Servo flipper;

    // Servo positions for 300-degree GoBilda servo (0.0 to 1.0 corresponds to 0° to 300°)
    // 90 degrees / 300 degrees = 0.30 position change
    private static final double POSITION_REST = 0.0;    // 0 degrees
    private static final double POSITION_FLIPPED = 0.3; // 90 degrees

    private boolean isFlipped = false;
    private boolean previousY = false;

    // Previous states for d-pad edge detection
    private boolean previousDpadUp = false;
    private boolean previousDpadDown = false;

    @Override
    public void init() {
        // Initialize the hardware using the mechanism class
        drivetrain.init(hardwareMap);
        pollenShooter.init(hardwareMap);

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Initialize flipper servo configured as "servo" in Driver Hub
        flipper = hardwareMap.get(Servo.class, "servo");
        flipper.setPosition(POSITION_REST);

        telemetry.addLine("Mecanum Drivetrain Initialized");
        telemetry.addLine("-----------------------------");
        telemetry.addLine("Controls:");
        telemetry.addLine("LS Y: Drive");
        telemetry.addLine("LS X: Strafe");
        telemetry.addLine("RS X: Rotate");
        telemetry.addLine("RT/LT: Intake");
        telemetry.addLine("Y Button: Toggle Flipper (90 deg)");
        telemetry.addLine("A Button: Run Shooter");
        telemetry.addLine("D-Pad Up/Down: Adjust Shooter RPM");
    }

    @Override
    public void loop() {
        // Read gamepad inputs
        // Note: Y axis is often inverted on controllers (up is negative)
        double drive  = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double rotate =  gamepad1.right_stick_x;
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        // Apply movement to the drivetrain
        drivetrain.drive(drive, strafe, rotate);

        // Control intake motor with right trigger
        intake.setPower(-intakePower);

        // Toggle flipper servo when Y button is pressed (edge detection)
        boolean currentY = gamepad1.y;
        if (currentY && !previousY) {
            isFlipped = !isFlipped;
            flipper.setPosition(isFlipped ? POSITION_FLIPPED : POSITION_REST);
        }
        previousY = currentY;

        // D-pad Up/Down to adjust Shooter1 speed by 100 rpm (edge detection)
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;

        if (currentDpadUp && !previousDpadUp) {
            pollenShooter.adjustRpm(100);
        }
        if (currentDpadDown && !previousDpadDown) {
            pollenShooter.adjustRpm(-100);
        }
        previousDpadUp = currentDpadUp;
        previousDpadDown = currentDpadDown;

        // Button A to run Shooter1 at target rpm
        if (gamepad1.a) {
            pollenShooter.runShooter();
        } else {
            pollenShooter.stopShooter();
        }

        // Display power levels and servo status for debugging/verification
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive Inputs", "D: %.2f, S: %.2f, R: %.2f", drive, strafe, rotate);
        telemetry.addData("Intake Power", "%.2f", intakePower);
        telemetry.addData("Flipper State", isFlipped ? "Flipped (90°)" : "Rest (0°)");
        telemetry.addData("Flipper Position", "%.2f", flipper.getPosition());
        telemetry.addData("Shooter Target RPM", pollenShooter.getTargetRpm());
        telemetry.addData("Shooter Real RPM","%.2f",pollenShooter.getCurrentRpm());

        telemetry.addData("Motor Powers", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", 
            drivetrain.getFrontLeftPower(),
            drivetrain.getFrontRightPower(),
            drivetrain.getBackLeftPower(),
            drivetrain.getBackRightPower());
    }

    @Override
    public void stop() {
        // Optional: Ensure motors are stopped when OpMode ends
        drivetrain.setMotorPowers(0, 0, 0, 0);
        pollenShooter.stopShooter();
        if (intake != null) {
            intake.setPower(0);
        }
    }
}
