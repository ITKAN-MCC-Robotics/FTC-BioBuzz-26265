package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Transfer Mechanism for Biobuzz robot.
 * Controls the motor "transferMotor" and servo "shooterPgate".
 * Includes current-based stall detection to prevent motor burnout and handle jams.
 */
public class TransferMechanism {
    public static final double DEFAULT_TRANSFER_POWER = 0.8;
    public static final double GATE_OPEN_POS = 0.53;
    public static final double GATE_CLOSED_POS = 0.66;

    // Stall detection settings
    public static final double STALL_CURRENT_THRESHOLD_AMPS = 5; // Current threshold in Amps
    public static final long STALL_DURATION_MS = 250;              // Filter duration in ms to prevent false positives from inrush current

    private DcMotorEx transferMotor;
    private Servo shooterPgate;
    private boolean gateOpen = false;

    // Stall detection state
    private boolean isStalled = false;
    private long stallStartTimeMs = 0;

    /**
     * Initializes hardware for transfer motor and shooter gate servo.
     * @param hwMap HardwareMap from OpMode
     */
    public void init(HardwareMap hwMap) {
        // Reset stall state on init
        resetStall();

        // Try hardware lookup for transfer motor with fallbacks (using DcMotorEx for current monitoring)
        transferMotor = getHardwareWithFallback(hwMap, DcMotorEx.class, "transferMotor");
        if (transferMotor != null) {
            transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Try hardware lookup for shooter gate servo with fallbacks
        shooterPgate = getHardwareWithFallback(hwMap, Servo.class, "shooterPgate");
        if (shooterPgate != null) {
            closeShooterGate();
        }
    }

    /**
     * Helper to retrieve hardware with fallback device names.
     */
    private <T> T getHardwareWithFallback(HardwareMap hwMap, Class<T> classOrInterface, String... names) {
        for (String name : names) {
            try {
                T device = hwMap.get(classOrInterface, name);
                if (device != null) {
                    return device;
                }
            } catch (Exception ignored) {
            }
        }
        return null;
    }

    /**
     * Periodically monitors motor current draw and updates stall state.
     * Should be called in the OpMode loop.
     */
    public void update() {
        if (transferMotor == null) return;

        double currentAmps = getMotorCurrent();

        // Monitor stall condition only when attempting to run motor forward
        if (!isStalled && transferMotor.getPower() > 0.0) {
            if (currentAmps >= STALL_CURRENT_THRESHOLD_AMPS) {
                if (stallStartTimeMs == 0) {
                    stallStartTimeMs = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - stallStartTimeMs >= STALL_DURATION_MS) {
                    // Sustained high current draw indicates mechanical stall
                    isStalled = true;
                    transferMotor.setPower(0.0);
                }
            } else {
                stallStartTimeMs = 0;
            }
        } else if (transferMotor.getPower() == 0.0 && !isStalled) {
            stallStartTimeMs = 0;
        }
    }

    /**
     * Sets explicit transfer motor power.
     * Blocked if the motor is currently stalled and attempting to run forward.
     * Reversing the motor automatically clears the stall condition.
     * @param power Power level (-1.0 to 1.0)
     */
    public void setTransferPower(double power) {
        // Reversing motor clears stall state
        if (power < 0.0) {
            resetStall();
        }

        // Block forward motor operation while stalled
        if (isStalled && power > 0.0) {
            if (transferMotor != null) {
                transferMotor.setPower(0.0);
            }
            return;
        }

        if (transferMotor != null) {
            transferMotor.setPower(power);
        }
    }

    /**
     * Resets the stall flag and timer, allowing the motor to run forward again.
     */
    public void resetStall() {
        isStalled = false;
        stallStartTimeMs = 0;
    }

    /**
     * Returns whether the transfer motor is currently in a stalled state.
     * @return true if stalled
     */
    public boolean isStalled() {
        return isStalled;
    }

    /**
     * Returns the live electrical current draw of the transfer motor in Amps.
     * @return current draw in Amps, or 0.0 if motor is unavailable
     */
    public double getMotorCurrent() {
        if (transferMotor != null) {
            try {
                return transferMotor.getCurrent(CurrentUnit.AMPS);
            } catch (Exception ignored) {
            }
        }
        return 0.0;
    }

    /**
     * Runs transfer forward to feed pollen balls into shooter.
     */
    public void runTransfer() {
        setTransferPower(DEFAULT_TRANSFER_POWER);
    }

    /**
     * Reverses transfer motor to clear pollen balls.
     */
    public void reverseTransfer() {
        setTransferPower(-DEFAULT_TRANSFER_POWER);
    }

    /**
     * Stops transfer motor.
     */
    public void stopTransfer() {
        setTransferPower(0.0);
    }

    /**
     * Opens shooter gate allowing pollen balls into flywheel.
     */
    public void openShooterGate() {
        setShooterGatePosition(GATE_OPEN_POS);
        gateOpen = true;
    }

    /**
     * Closes shooter gate blocking pollen balls from entering flywheel.
     */
    public void closeShooterGate() {
        setShooterGatePosition(GATE_CLOSED_POS);
        gateOpen = false;
    }

    /**
     * Sets shooter gate servo position.
     * @param position Servo position (0.0 to 1.0)
     */
    public void setShooterGatePosition(double position) {
        if (shooterPgate != null) {
            shooterPgate.setPosition(position);
        }
    }

    /**
     * Returns whether shooter gate is currently open.
     * @return true if open
     */
    public boolean isShooterGateOpen() {
        return gateOpen;
    }

    /**
     * Checks if the shooter gate servo hardware is initialized.
     * @return true if servo is non-null
     */
    public boolean isServoAttached() {
        return shooterPgate != null;
    }

    /**
     * Returns transfer motor power.
     * @return current power
     */
    public double getTransferPower() {
        return transferMotor != null ? transferMotor.getPower() : 0.0;
    }

    /**
     * Returns shooter gate servo position.
     * @return current position
     */
    public double getShooterGatePosition() {
        return shooterPgate != null ? shooterPgate.getPosition() : 0.0;
    }
}

