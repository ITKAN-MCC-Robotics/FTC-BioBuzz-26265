package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Basic PedroPathing autonomous that just moves 10 units forward.
 * Stripped of all other mechanisms (intake, shooter, Limelight).
 */
@Autonomous(name = "TestAuto Forward 10", group = "Autonomous")
public class TestAuto extends OpMode {

    // Pedro
    private Follower follower;
    private Timer stepTimer;

    // Start pose: (X=0, Y=0, Heading=0)
    // Moving 10 units forward means going to X=10.
    private static final Pose START_POSE = new Pose(0.0, 0.0, 0.0);

    // Paths
    private Paths paths;

    // State machine steps
    private enum Step {
        DRIVE_FORWARD,
        WAIT_FOR_DRIVE,
        DONE
    }

    private Step step;

    private void go(Step next) {
        step = next;
        stepTimer.resetTimer();
    }

    private void follow(PathChain p, Step nextAfterStart) {
        follower.followPath(p, true);
        go(nextAfterStart);
    }

    @Override
    public void init() {
        stepTimer = new Timer();
        
        // Initialize the PedroPathing follower
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        // Seed pose + build paths
        follower.setPose(START_POSE);
        paths = new Paths(follower);

        // Set the initial step
        go(Step.DRIVE_FORWARD);
    }

    @Override
    public void loop() {
        // Update PedroPathing follower every loop
        follower.update();

        switch (step) {
            case DRIVE_FORWARD:
                // Follow the path to move forward, then transition to WAIT_FOR_DRIVE
                follow(paths.Path1, Step.WAIT_FOR_DRIVE);
                break;

            case WAIT_FOR_DRIVE:
                // Wait until the follower reaches the target
                if (!follower.isBusy()) {
                    go(Step.DONE);
                }
                break;

            case DONE:
                // Autonomous is complete, do nothing
                break;
        }

        // Minimal Telemetry
        Pose p = follower.getPose();
        telemetry.addData("Step", step);
        telemetry.addData("Pose", "X %.1f  Y %.1f  H %.1f",
                p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Nothing special to stop for pure movement
    }

    // ============================
    // Simple 10-unit forward Path
    // ============================
    public static class Paths {
        public PathChain Path1;

        public Paths(Follower follower) {
            // Move 10 units forward on the X axis while maintaining a 0-degree heading.
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(0.0, 0.0, 0.0),
                                    new Pose(10.0, 0.0, 0.0)
                            )
                    )
                    .setConstantHeadingInterpolation(0.0)
                    .build();
        }
    }
}
