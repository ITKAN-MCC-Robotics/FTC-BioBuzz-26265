package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrainMechanism;

/**
 * DriveOnlyOpMode
 * TeleOp mode for pure Mecanum drivetrain control without intake, shooter, or auxiliary mechanisms.
 * Uses Gamepad 1 for driving:
 * - Left Stick Y: Forward/Backward
 * - Left Stick X: Strafe Left/Right
 * - Right Stick X: Rotate Clockwise/Counter-clockwise
 */
@TeleOp(name = "Drive Only OpMode", group = "TeleOp")
public class DriveOnlyOpMode extends OpMode {
    // Instantiate the drivetrain mechanism
    private final DriveTrainMechanism drivetrain = new DriveTrainMechanism();

    @Override
    public void init() {
        // Initialize the hardware using the mechanism class
        drivetrain.init(hardwareMap);

        telemetry.addLine("Drive Only OpMode Initialized");
        telemetry.addLine("-----------------------------");
        telemetry.addLine("Controls:");
        telemetry.addLine("LS Y: Drive");
        telemetry.addLine("LS X: Strafe");
        telemetry.addLine("RS X: Rotate");
    }

    @Override
    public void loop() {
        // Read gamepad inputs
        // Note: Y axis is inverted on controllers (up is negative)
        double drive  = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double rotate =  gamepad1.right_stick_x;

        // Apply movement to the drivetrain
        drivetrain.drive(drive, strafe, rotate);

        // Display power levels and inputs for debugging/verification
        telemetry.addData("Status", "Running");
        telemetry.addData("Drive Inputs", "D: %.2f, S: %.2f, R: %.2f", drive, strafe, rotate);
        telemetry.addData("Motor Powers", "FL: %.2f, FR: %.2f, BL: %.2f, BR: %.2f", 
            drivetrain.getFrontLeftPower(),
            drivetrain.getFrontRightPower(),
            drivetrain.getBackLeftPower(),
            drivetrain.getBackRightPower());
    }

    @Override
    public void stop() {
        // Ensure motors are stopped when OpMode ends
        drivetrain.setMotorPowers(0, 0, 0, 0);
    }
}
