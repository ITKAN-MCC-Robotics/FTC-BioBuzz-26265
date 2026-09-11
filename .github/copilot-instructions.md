# FTC Java Coding Agent Instructions (Command-Based & FTCLib)

You are an expert FIRST Tech Challenge (FTC) Java programming assistant working inside Android Studio. Follow these architecture rules, naming standards, and conventions whenever generating, refactoring, or reviewing robot code for FTC teams using **FTCLib**.

---

## 1. Core Rule: OpMode Over LinearOpMode

* **Default Superclass:** Always extend `com.qualcomm.robotcore.eventloop.opmode.OpMode`.
  * When using FTCLib’s command-based framework, extend `com.arcrobotics.ftclib.command.CommandOpMode` (which bridges FTC lifecycles to FTCLib's command scheduler).
* **Never use bare `LinearOpMode`:** Do not generate standard `extends LinearOpMode`, `waitForStart()`, `opModeIsActive()`, or blocking `sleep(...)` statements.
* **Lifecycle Method Rules:**
  * `initialize()` (in `CommandOpMode`) / `init()` (in `OpMode`):
    * Initialize hardware devices, instantiate subsystems, configure GamepadEx objects, bind buttons to commands, set default subsystem commands, and initialize mechanism positions.
  * `init_loop()`:
    * Non-blocking pre-match tasks (e.g., vision targeting, sensor stabilization, alliance selection telemetry).
  * `run()` (in `CommandOpMode`) / `loop()` (in `OpMode`):
    * Main periodic execution (~50 Hz). Must remain strictly non-blocking.
    * In `CommandOpMode`, **always** invoke `super.run()` so `CommandScheduler.getInstance().run()` is ticked every loop iteration.
  * `reset()` / `stop()`:
    * Cancel active commands, shut down motor powers to 0.0, and release vision streams (`VisionPortal.close()`).

---

## 2. Command-Based Architecture & Partitioning

Strictly partition code across three main packages under `org.firstinspires.ftc.teamcode`:

### A. Subsystems (`org.firstinspires.ftc.teamcode.subsystems`)
* All physical mechanisms (e.g., `DriveSubsystem`, `IntakeSubsystem`, `LiftSubsystem`, `ArmSubsystem`, `ClawSubsystem`) must extend `com.arcrobotics.ftclib.command.SubsystemBase`.
* **Hardware Encapsulation:** All hardware instances (`MotorEx`, `DcMotor`, `Servo`, `DigitalChannel`, `IMU`, `TouchSensor`) must be **`private`**.
* **Periodic Updates:** Implement `periodic()` to poll sensors, update odometry localizers, run PID loop calculations, and push subsystem state to telemetry.
* **Isolation Rule:** Subsystems must **never** read gamepad instances or manage command lifecycles directly. They must only expose high-level physical actions (e.g., `drive(double forward, double strafe, double turn)`, `setSlidePosition(int targetTicks)`, `openClaw()`, `stop()`).

### B. Commands (`org.firstinspires.ftc.teamcode.commands`)
* Custom commands must extend `com.arcrobotics.ftclib.command.CommandBase` or utilize FTCLib inline commands (`InstantCommand`, `RunCommand`, `WaitCommand`, `FunctionalCommand`, `StartEndCommand`).
* **Subsystem Requirements:** Always call `addRequirements(subsystem1, subsystem2...)` in the constructor so `CommandScheduler` can interrupt conflicting commands targeting the same hardware.
* **Command Lifecycle:**
  * `initialize()`: Runs once when scheduled (resets timers, captures initial sensor zero points).
  * `execute()`: Runs periodically while active (~50 Hz).
  * `end(boolean interrupted)`: Cleans up mechanism states (e.g., stops motor power, holds target position).
  * `isFinished()`: Returns `true` when target conditions are met (e.g., target position reached within tolerance or timer expired); returns `false` for perpetual or default driver commands.
* **Command Composition:** Group complex autonomous or TeleOp sequences using `SequentialCommandGroup`, `ParallelCommandGroup`, `ParallelRaceGroup`, or `ParallelDeadlineGroup`.

### C. OpModes (`org.firstinspires.ftc.teamcode.opmodes`)
* Annotate with `@TeleOp(name = "...", group = "...")` or `@Autonomous(name = "...", group = "...")`.
* Act as a thin orchestration layer:
  1. Instantiate subsystems and hardware mappings.
  2. Instantiate `GamepadEx` driver controllers.
  3. Bind gamepad buttons/triggers to commands.
  4. Set subsystem default commands (e.g., `driveSubsystem.setDefaultCommand(...)`).
  5. Schedule autonomous command sequences using `schedule(...)`.

---

## 3. Gamepad Handling via FTCLib `GamepadEx`

* **Avoid Standard Gamepad Polling:** Replace raw `gamepad1` / `gamepad2` polling with `com.arcrobotics.ftclib.gamepad.GamepadEx`.
* **Automatic Stick Inversion Notice:**
  * FTCLib's `gamepadEx.getLeftY()` and `gamepadEx.getRightY()` **automatically invert** the FTC SDK stick Y-polarity (pushing forward returns `+1.0`, pulling backward returns `-1.0`).
  * **Do not double-invert** stick Y values when passing them to drive or motor commands.
* **Declarative Button Binding:**
  * Wrap controller buttons in `com.arcrobotics.ftclib.command.button.GamepadButton` using `com.arcrobotics.ftclib.gamepad.GamepadKeys.Button`.
  * Bind commands declaratively inside `initialize()`:
    * `.whenPressed(...)` / `.onTrue(...)`: Triggers a command once when the button transitions to pressed.
    * `.whileHeld(...)` / `.whileTrue(...)`: Runs a command continuously while held, cancels on release.
    * `.toggleWhenPressed(...)`: Toggles between two commands or states on each press.
    * `.whenReleased(...)` / `.onFalse(...)`: Triggers a command on button release.
* **Analog Trigger Threshold Triggers:**
  * Wrap analog triggers (`gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)`) into a `com.arcrobotics.ftclib.command.button.Trigger` using a threshold predicate (e.g., `new Trigger(() -> gamepadEx.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5)`).
* **Edge Detection Without Command Scheduler:**
  * If writing non-command OpMode logic, use FTCLib reader classes (`ButtonReader`, `TriggerReader`, `ToggleButtonReader`).
  * Always call `reader.readValue()` once per loop iteration before querying `wasJustPressed()`, `wasJustReleased()`, or `stateJustChanged()`.

---

## 4. Drivetrain Configuration & Motor Standards

* **Hardware Mapping Names:** Standardize 4-motor drivetrains with these exact configuration strings:
  1. `"front_left"`
  2. `"front_right"`
  3. `"back_left"`
  4. `"back_right"`
* **Setup Best Practices:**
  * Invert left-side motors so positive power commands move the robot forward.
  * Always set `ZeroPowerBehavior.BRAKE` for precise stopping and position holding.
  * Use `RUN_USING_ENCODER` (or FTCLib `Motor.RunMode.VelocityControl`) to maintain consistent velocity despite battery voltage drop.
* **Power Normalization Algorithm:**
  * In all Mecanum or Holonomic kinematics calculations, scale all motor powers proportionally whenever any calculated power exceeds `1.0`:
  ```java
  double maxPower = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
  maxPower = Math.max(maxPower, Math.abs(backLeft));
  maxPower = Math.max(maxPower, Math.abs(backRight));

  if (maxPower > 1.0) {
      frontLeft /= maxPower;
      frontRight /= maxPower;
      backLeft /= maxPower;
      backRight /= maxPower;
  }
  ```

---

## 5. Advanced Autonomous Navigation & Localization

* **Odometry & Dead-Wheel Tracking:**
  * Manage dead-wheel tracking pods via FTCLib's `HolonomicOdometry` or custom odometry subsystems.
  * Keep robot dimensions, wheel track width, pod offsets, and gearing in centralized constant classes (`final double TRACKWIDTH`, `final double CENTER_WHEEL_OFFSET`, `final double TICKS_PER_REV`).
  * Execute `odometry.updatePose(...)` inside `DriveSubsystem.periodic()` so location tracking is continuous across both TeleOp and Autonomous modes.
* **Trajectory Following & Pathing:**
  * Use non-blocking trajectory commands (e.g., FTCLib's `MecanumControllerCommand`, `PurePursuitCommand`, or integrate Pedro Pathing / Road Runner pathing followers).
  * Build complete autonomous routines by composing command groups rather than hardcoding timer delays or state-machine ticks.
* **Sensor & Vision Integration:**
  * **IMU (Gyro):** Initialize Control Hub orientation using `RevHubOrientationOnRobot` (FTC SDK 8.1+) and extract heading using normalized angles (`AngleUnit.DEGREES` or `AngleUnit.RADIANS`).
  * **Units:** Always wrap measurements in `DistanceUnit` (`CM`, `INCH`) and `AngleUnit` to avoid conversion errors.
  * **Limit Switches / Touch Sensors:** Use `!touchSensor.getState()` so pressed evaluates to `true` (standard digital channels pull LOW when pressed).
  * **Vision:** Use modern `VisionPortal` with `AprilTagProcessor` or custom `VisionProcessor`. Ensure streams are paused/closed when inactive to save CPU/GPU cycles on the REV Control Hub.

---

## 6. Commenting & Readability Standards for FTC Students

Every generated class, method, and variable must remain educational and accessible for FTC middle and high school students:

* **Class-Level Javadoc:** Document the physical mechanism, hardware constraints, subsystem dependencies, and game strategy represented by the class.
* **Field Comments:** Document every member variable, explicitly listing hardware map names, engineering units (e.g., `encoder ticks`, `seconds`, `cm`, `volts`, `ratio -1.0 to +1.0`), and physical limits.
* **The "Why" Rule:** Comment *why* a specific calculation, PID coefficient, mechanical gear ratio, or power reduction is applied, rather than echoing Java syntax.
* **Naming Conventions:**
  * Use `camelCase` for variables and methods (`intakeMotor`, `isClawClosed`, `getTargetHeading()`).
  * Use `UPPER_SNAKE_CASE` for class constants (`TICKS_PER_INCH`, `MAX_VELOCITY_DEG_PER_SEC`).
  * Prefix commands with descriptive verbs (`DriveToAprilTagCommand`, `LiftToPositionCommand`).

---

## 7. Reference Implementations

### A. Subsystem Pattern (`DriveSubsystem.java`)
```java
package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem responsible for controlling a 4-motor Mecanum drivetrain.
 * Handles motor control, velocity normalization, and periodic odometry updates.
 */
public class DriveSubsystem extends SubsystemBase {
    private final MotorEx frontLeft;
    private final MotorEx frontRight;
    private final MotorEx backLeft;
    private final MotorEx backRight;

    /**
     * Initializes drivetrain motors from the FTC HardwareMap.
     * @param hwMap FTC OpMode hardware map instance.
     */
    public DriveSubsystem(HardwareMap hwMap) {
        frontLeft = new MotorEx(hwMap, "front_left");
        frontRight = new MotorEx(hwMap, "front_right");
        backLeft = new MotorEx(hwMap, "back_left");
        backRight = new MotorEx(hwMap, "back_right");

        // Invert left motors so positive command values drive forward
        frontLeft.setInverted(true);
        backLeft.setInverted(true);

        // Set zero power behavior to BRAKE for quick stopping
        frontLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drives the robot using mecanum kinematic calculations.
     * @param drive Forward/backward velocity (-1.0 to 1.0)
     * @param strafe Left/right velocity (-1.0 to 1.0)
     * @param turn Clockwise/counter-clockwise turn rate (-1.0 to 1.0)
     */
    public void driveRobotCentric(double drive, double strafe, double turn) {
        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        // Normalize motor powers if any exceeds 1.0
        double max = Math.max(Math.abs(fl), Math.abs(fr));
        max = Math.max(max, Math.abs(bl));
        max = Math.max(max, Math.abs(br));

        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.set(fl);
        frontRight.set(fr);
        backLeft.set(bl);
        backRight.set(br);
    }

    /**
     * Halts all drivetrain motor power.
     */
    public void stop() {
        driveRobotCentric(0, 0, 0);
    }

    @Override
    public void periodic() {
        // Update odometry, log telemetry, or handle periodic subsystem tasks here
    }
}
```

### B. Command Pattern (`DefaultDriveCommand.java`)
```java
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Default command for driving the robot in TeleOp using gamepad joystick inputs.
 */
public class DefaultDriveCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier driveSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier turnSupplier;

    public DefaultDriveCommand(
            DriveSubsystem driveSubsystem,
            DoubleSupplier driveSupplier,
            DoubleSupplier strafeSupplier,
            DoubleSupplier turnSupplier
    ) {
        this.driveSubsystem = driveSubsystem;
        this.driveSupplier = driveSupplier;
        this.strafeSupplier = strafeSupplier;
        this.turnSupplier = turnSupplier;

        // Declare that this command requires the DriveSubsystem
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.driveRobotCentric(
                driveSupplier.getAsDouble(),
                strafeSupplier.getAsDouble(),
                turnSupplier.getAsDouble()
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
```

### C. OpMode Orchestration Pattern (`MainTeleOp.java`)
```java
package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends CommandOpMode {
    private DriveSubsystem driveSubsystem;
    private GamepadEx driverGamepad;

    @Override
    public void initialize() {
        // 1. Initialize Subsystems
        driveSubsystem = new DriveSubsystem(hardwareMap);

        // 2. Initialize Gamepads
        driverGamepad = new GamepadEx(gamepad1);

        // 3. Set Default Commands (Note: GamepadEx.getLeftY() is already inverted forward)
        driveSubsystem.setDefaultCommand(new DefaultDriveCommand(
                driveSubsystem,
                () -> driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX()
        ));

        // 4. Bind Buttons to Commands
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(driveSubsystem::stop, driveSubsystem));
    }
}
```
