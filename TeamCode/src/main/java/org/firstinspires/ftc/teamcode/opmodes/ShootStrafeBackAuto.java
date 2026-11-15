package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.PollenShooterMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.TransferMechanism;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autonomous OpMode for Biobuzz 26265.
 * 
 * Autonomous Sequence:
 * 1. Run pollen shooting sequence for 4.0 seconds (spin shooter flywheel, open gate, run transfer motor).
 * 2. Strafe right by 30 inches using PedroPathing closed-loop odometry.
 * 3. Move backwards by 70 inches using PedroPathing closed-loop odometry.
 */
@Autonomous(name = "Shoot Strafe Back Auto", group = "Autonomous")
public class ShootStrafeBackAuto extends OpMode {

    // Pedro Pathing follower & step timer
    private Follower follower;
    private Timer stepTimer;

    // Active Mechanisms
    private final PollenShooterMechanism shooter = new PollenShooterMechanism();
    private final TransferMechanism transfer = new TransferMechanism();

    // Poses defined in Pedro Pathing coordinates (inches, radians)
    // Starting pose: X = 0.0, Y = 0.0, Heading = 0.0°
    private static final Pose START_POSE = new Pose(0.0, 0.0, Math.toRadians(0.0));
    // Strafe Right 30 inches: Y moves -30.0 inches from start
    private static final Pose STRAFE_RIGHT_TARGET = new Pose(0.0, -30.0, Math.toRadians(0.0));
    // Move Backwards 70 inches: X moves -70.0 inches from (0, -30) -> (-70, -30)
    private static final Pose BACKWARDS_TARGET = new Pose(-70.0, -30.0, Math.toRadians(0.0));

    // State Machine Steps
    private enum Step {
        SHOOT_SEQUENCE,
        START_STRAFE_RIGHT,
        WAIT_FOR_STRAFE_RIGHT,
        START_MOVE_BACKWARDS,
        WAIT_FOR_MOVE_BACKWARDS,
        DONE
    }

    private Step step;

    private void go(Step nextStep) {
        step = nextStep;
        stepTimer.resetTimer();
    }

    @Override
    public void init() {
        stepTimer = new Timer();

        // 1. Hardware-level Pinpoint Odometry reset (zeroes IMU heading & pod registers)
        try {
            GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            if (pinpoint != null) {
                pinpoint.resetPosAndIMU();
            }
        } catch (Exception ignored) {
        }

        // 2. Initialize PedroPathing follower and seed starting pose
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);

        // 3. Initialize shooter and transfer mechanisms
        shooter.init(hardwareMap);
        transfer.init(hardwareMap);

        // Set initial state to shoot sequence
        go(Step.SHOOT_SEQUENCE);

        telemetry.addLine("Shoot Strafe Back Auto Initialized");
        telemetry.addData("Starting Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                START_POSE.getX(), START_POSE.getY(), Math.toDegrees(START_POSE.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Reset step timer and ensure start pose is set on match start
        if (follower != null) {
            follower.setPose(START_POSE);
        }
        go(Step.SHOOT_SEQUENCE);
    }

    @Override
    public void loop() {
        // Update follower odometry calculations and mechanism states
        follower.update();
        transfer.update();
        shooter.update();

        Pose currentPose = follower.getPose();

        switch (step) {
            case SHOOT_SEQUENCE:
                // 1. Enable shooter flywheel using PollenShooterMechanism's configured target RPM
                shooter.setFlywheelOn(true);

                // 2. Open gate and feed transfer motor once shooter reaches target speed
                if (shooter.isAtTargetSpeed()) {
                    transfer.openShooterGate();
                    transfer.runTransfer();
                } else {
                    transfer.closeShooterGate();
                    transfer.stopTransfer();
                }

                // 3. Run shooting sequence for 4.0 seconds total
                if (stepTimer.getElapsedTimeSeconds() >= 4.0) {
                    // Stop shooting mechanisms
                    transfer.stopTransfer();
                    transfer.closeShooterGate();
                    shooter.setFlywheelOn(false);
                    shooter.stopShooter();

                    // Transition to strafe right step
                    go(Step.START_STRAFE_RIGHT);
                }
                break;

            case START_STRAFE_RIGHT:
                // Dynamically build path from current robot pose to strafe target (-30 in Y)
                PathChain pathStrafeRight = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, STRAFE_RIGHT_TARGET))
                        .setConstantHeadingInterpolation(START_POSE.getHeading())
                        .build();

                follower.followPath(pathStrafeRight, true);
                go(Step.WAIT_FOR_STRAFE_RIGHT);
                break;

            case WAIT_FOR_STRAFE_RIGHT:
                if (!follower.isBusy()) {
                    go(Step.START_MOVE_BACKWARDS);
                }
                break;

            case START_MOVE_BACKWARDS:
                // Dynamically build path from current robot pose to backwards target (-70 in X, -30 in Y)
                PathChain pathMoveBackwards = follower.pathBuilder()
                        .addPath(new BezierLine(currentPose, BACKWARDS_TARGET))
                        .setConstantHeadingInterpolation(START_POSE.getHeading())
                        .build();

                follower.followPath(pathMoveBackwards, true);
                go(Step.WAIT_FOR_MOVE_BACKWARDS);
                break;

            case WAIT_FOR_MOVE_BACKWARDS:
                if (!follower.isBusy()) {
                    go(Step.DONE);
                }
                break;

            case DONE:
                // Autonomous routine completed; maintain active position holding at target pose
                break;
        }

        // Telemetry feedback for driver station
        telemetry.addData("Step", step);
        telemetry.addData("Timer (s)", "%.2f", stepTimer.getElapsedTimeSeconds());
        telemetry.addData("Pedro Pose (in)", "X: %.2f, Y: %.2f, Heading: %.1f°",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Follower Busy", follower.isBusy() ? "YES" : "NO");
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRpm(), shooter.getTargetRpm());
        telemetry.addData("Transfer State", transfer.isShooterGateOpen() ? "FEEDING" : "STOPPED");
        telemetry.update();
    }

    @Override
    public void stop() {
        transfer.stopTransfer();
        transfer.closeShooterGate();
        shooter.setFlywheelOn(false);
        shooter.stopShooter();
        if (follower != null) {
            follower.breakFollowing();
        }
    }
}
