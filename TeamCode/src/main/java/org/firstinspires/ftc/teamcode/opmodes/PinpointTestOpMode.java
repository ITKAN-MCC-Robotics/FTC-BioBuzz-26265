package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrainMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointMechanism;

/**
 * PinpointTestOpMode
 *
 * Dedicated OpMode to test and verify GoBilda Pinpoint Odometry localization.
 *
 * Controls:
 * - Left & Right Sticks: Manual drive controls to test position tracking.
 * - Gamepad A Button   : Reset to Red Audience Wall 2nd Tile (-36", -63", 90°)
 * - Gamepad B Button   : Reset to Blue Audience Wall 2nd Tile (36", -63", 90°)
 * - Gamepad X Button   : Set position to Red Audience Wall 2nd Tile (-36", -63", 90°)
 * - Gamepad Y Button   : Set position to Blue Audience Wall 2nd Tile (36", -63", 90°)
 */
@TeleOp(name = "Pinpoint Test OpMode", group = "Testing")
public class PinpointTestOpMode extends OpMode {

    private final DriveTrainMechanism drivetrain = new DriveTrainMechanism();
    private final PinpointMechanism pinpoint = new PinpointMechanism();

    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevY = false;

    @Override
    public void init() {
        drivetrain.init(hardwareMap);
        pinpoint.init(hardwareMap);

        telemetry.addLine("GoBilda Pinpoint Localization Test Initialized");
        telemetry.addLine("---------------------------------------------");
        telemetry.addLine("[A] Reset Red Audience Wall 2nd Tile (-36, -63, 90°)");
        telemetry.addLine("[B] Reset Blue Audience Wall 2nd Tile (36, -63, 90°)");
        telemetry.addLine("[X] Set Red Starting Pose (-36, -63, 90°)");
        telemetry.addLine("[Y] Set Blue Starting Pose (36, -63, 90°)");
    }

    @Override
    public void loop() {
        // Update pinpoint computer
        pinpoint.update();

        // 1. Manual Drive Control
        double drive  = -gamepad1.left_stick_y;
        double strafe =  gamepad1.left_stick_x;
        double rotate =  gamepad1.right_stick_x;
        drivetrain.drive(drive, strafe, rotate);

        // 2. Position Resets & Calibration
        boolean currentA = gamepad1.a;
        if (currentA && !prevA) {
            pinpoint.resetToAudienceWallRed();
        }
        prevA = currentA;

        boolean currentB = gamepad1.b;
        if (currentB && !prevB) {
            pinpoint.resetToAudienceWallBlue();
        }
        prevB = currentB;

        boolean currentX = gamepad1.x;
        if (currentX && !prevX) {
            pinpoint.resetToAudienceWallRed();
        }
        prevX = currentX;

        boolean currentY = gamepad1.y;
        if (currentY && !prevY) {
            pinpoint.resetToAudienceWallBlue();
        }
        prevY = currentY;

        // 3. Telemetry Output
        telemetry.addData("=== PINPOINT SENSOR STATUS ===", pinpoint.getDeviceStatus());
        telemetry.addData("Initialized", pinpoint.isInitialized() ? "YES" : "NO");
        telemetry.addLine();

        telemetry.addData("=== FTC FIELD POSITION ===", "");
        telemetry.addData("X (inches)", "%.2f", pinpoint.getX(DistanceUnit.INCH));
        telemetry.addData("Y (inches)", "%.2f", pinpoint.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.1f°", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();

        telemetry.addData("=== PEDRO PATHING POSITION ===", "");
        telemetry.addData("Pedro X (inches)", "%.2f", pinpoint.getPedroX());
        telemetry.addData("Pedro Y (inches)", "%.2f", pinpoint.getPedroY());
        telemetry.addLine();

        telemetry.addData("=== TURRET TARGETING ANGLES ===", "");
        telemetry.addData("Angle to Red Target", "%.1f°", Math.toDegrees(pinpoint.getAngleToRedTarget()));
        telemetry.addData("Angle to Blue Target", "%.1f°", Math.toDegrees(pinpoint.getAngleToBlueTarget()));
    }

    @Override
    public void stop() {
        drivetrain.setMotorPowers(0, 0, 0, 0);
    }
}
