package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Autoaim;
import org.firstinspires.ftc.teamcode.mechanisms.IntakeMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.PollenShooterMechanism;
import org.firstinspires.ftc.teamcode.mechanisms.TransferMechanism;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * RedAuto1 Autonomous Program for RED Alliance.
 * 
 * Sequence:
 * 1. Start at Red Alliance starting pose (Pedro: 36, 9, 270°).
 * 2. Move to (36, 15, 230°).
 * 3. Run pollen shooting sequence with autoaim for 4 seconds.
 * 4. Move to position (36, 36, 180°).
 * 5. Move to position (30, 110, 180°).
 * 6. Park at position (9, 115, 180°).
 */
@Autonomous(name = "RedAuto1", group = "Autonomous")
public class RedAuto1 extends OpMode {

    // Pedro Pathing follower & state timer
    private Follower follower;
    private Timer stepTimer;

    // Mechanisms
    private final PollenShooterMechanism shooter = new PollenShooterMechanism();
    private final TransferMechanism transfer = new TransferMechanism();
    private final IntakeMechanism intake = new IntakeMechanism();
    private final Autoaim autoaim = new Autoaim();

    // Poses defined in Pedro Pathing coordinates
    private static final Pose START_POSE = new Pose(36.0, 9.0, Math.toRadians(270.0));
    private static final Pose SHOOT_POSE = new Pose(36.0, 15.0, Math.toRadians(230.0));
    private static final Pose POSE_36_36 = new Pose(36.0, 36.0, Math.toRadians(180.0));
    private static final Pose POSE_30_110 = new Pose(30.0, 110.0, Math.toRadians(180.0));
    private static final Pose PARK_POSE = new Pose(9.0, 115.0, Math.toRadians(180.0));

    // Paths
    private Paths paths;

    // State Machine Steps
    private enum Step {
        MOVE_UP_AND_TURN,
        WAIT_FOR_MOVE_UP,
        SHOOT_SEQUENCE,
        MOVE_TO_36_36,
        WAIT_FOR_36_36,
        MOVE_TO_30_110,
        WAIT_FOR_30_110,
        PARK_9_115,
        WAIT_FOR_PARK,
        DONE
    }

    private Step step;

    private void go(Step nextStep) {
        step = nextStep;
        stepTimer.resetTimer();
    }

    private void follow(PathChain pathChain, Step nextStep) {
        follower.followPath(pathChain, true);
        go(nextStep);
    }

    @Override
    public void init() {
        stepTimer = new Timer();

        // Initialize follower and mechanisms
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);

        shooter.init(hardwareMap);
        transfer.init(hardwareMap);
        intake.init(hardwareMap);
        autoaim.init(hardwareMap);

        // Pre-build paths
        paths = new Paths(follower);

        go(Step.MOVE_UP_AND_TURN);

        telemetry.addLine("RedAuto1 Initialized");
        telemetry.addData("Starting Pose", "X: %.1f, Y: %.1f, H: %.1f°",
                START_POSE.getX(), START_POSE.getY(), Math.toDegrees(START_POSE.getHeading()));
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update Pedro follower and active mechanisms
        follower.update();
        transfer.update();
        shooter.update();

        Pose currentPose = follower.getPose();

        switch (step) {
            case MOVE_UP_AND_TURN:
                // Move to position (36, 15, 230°)
                follow(paths.path1MoveUpAndTurn, Step.WAIT_FOR_MOVE_UP);
                break;

            case WAIT_FOR_MOVE_UP:
                if (!follower.isBusy()) {
                    // Start shooter flywheel when arriving at shooting pose
                    shooter.setTargetRpm(PollenShooterMechanism.DEFAULT_RPM);
                    shooter.setFlywheelOn(true);
                    go(Step.SHOOT_SEQUENCE);
                }
                break;

            case SHOOT_SEQUENCE:
                // Run autoaim and transfer feed for 4 seconds
                autoaim.update(true, false, currentPose.getX(), currentPose.getY(), currentPose.getHeading(), false);
                
                // Open gate and feed transfer motor once shooter reaches speed or immediately
                transfer.openShooterGate();
                transfer.runTransfer();

                if (stepTimer.getElapsedTimeSeconds() >= 4.0) {
                    // Stop shooting mechanisms after 4 seconds
                    transfer.stopTransfer();
                    transfer.closeShooterGate();
                    shooter.setFlywheelOn(false);
                    shooter.stopShooter();
                    autoaim.resetPosition();

                    go(Step.MOVE_TO_36_36);
                }
                break;

            case MOVE_TO_36_36:
                // Move to position (36, 36, 180°)
                follow(paths.path2MoveTo36_36, Step.WAIT_FOR_36_36);
                break;

            case WAIT_FOR_36_36:
                if (!follower.isBusy()) {
                    go(Step.MOVE_TO_30_110);
                }
                break;

            case MOVE_TO_30_110:
                // Move to position (30, 110, 180°)
                follow(paths.path3MoveTo30_110, Step.WAIT_FOR_30_110);
                break;

            case WAIT_FOR_30_110:
                if (!follower.isBusy()) {
                    go(Step.PARK_9_115);
                }
                break;

            case PARK_9_115:
                // Park at position (9, 115, 180°)
                follow(paths.path4ParkAt9_115, Step.WAIT_FOR_PARK);
                break;

            case WAIT_FOR_PARK:
                if (!follower.isBusy()) {
                    go(Step.DONE);
                }
                break;

            case DONE:
                // Autonomous program complete
                follower.breakFollowing();
                break;
        }

        // Telemetry feedback
        telemetry.addData("Step", step);
        telemetry.addData("Timer (s)", "%.2f", stepTimer.getElapsedTimeSeconds());
        telemetry.addData("Pedro Pose", "X: %.2f, Y: %.2f, Heading: %.1f°",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Shooter RPM", "%.0f / %.0f", shooter.getCurrentRpm(), shooter.getTargetRpm());
        telemetry.addData("Transfer State", transfer.isShooterGateOpen() ? "FEEDING" : "STOPPED");
        telemetry.update();
    }

    @Override
    public void stop() {
        transfer.stopTransfer();
        transfer.closeShooterGate();
        shooter.stopShooter();
        autoaim.resetPosition();
    }

    /**
     * Path Definitions for RedAuto1
     */
    public static class Paths {
        public PathChain path1MoveUpAndTurn;
        public PathChain path2MoveTo36_36;
        public PathChain path3MoveTo30_110;
        public PathChain path4ParkAt9_115;

        public Paths(Follower follower) {
            // Path 1: Start (36, 9, 270°) -> (36, 15, 230°)
            path1MoveUpAndTurn = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // Path 2: Shoot Pose (36, 15, 230°) -> (36, 36, 180°)
            path2MoveTo36_36 = follower.pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE, POSE_36_36))
                    .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), POSE_36_36.getHeading())
                    .build();

            // Path 3: (36, 36, 180°) -> (30, 110, 180°)
            path3MoveTo30_110 = follower.pathBuilder()
                    .addPath(new BezierLine(POSE_36_36, POSE_30_110))
                    .setLinearHeadingInterpolation(POSE_36_36.getHeading(), POSE_30_110.getHeading())
                    .build();

            // Path 4: (30, 110, 180°) -> Park at (9, 115, 180°)
            path4ParkAt9_115 = follower.pathBuilder()
                    .addPath(new BezierLine(POSE_30_110, PARK_POSE))
                    .setLinearHeadingInterpolation(POSE_30_110.getHeading(), PARK_POSE.getHeading())
                    .build();
        }
    }
}
