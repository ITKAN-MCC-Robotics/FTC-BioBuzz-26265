package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Transfer Mechanism for Biobuzz robot.
 * Controls the motor "Transfer" and servo "Shooter1Gate".
 */
public class TransferMechanism {
    public static final double DEFAULT_TRANSFER_POWER = 1.0;
    public static final double GATE_OPEN_POS = 1.0;
    public static final double GATE_CLOSED_POS = 0.0;

    private DcMotor transferMotor;
    private Servo shooter1Gate;
    private boolean gateOpen = false;

    /**
     * Initializes hardware for transfer motor and shooter gate servo.
     * @param hwMap HardwareMap from OpMode
     */
    public void init(HardwareMap hwMap) {
        transferMotor = hwMap.get(DcMotor.class, "Transfer");
        shooter1Gate = hwMap.get(Servo.class, "Shooter1Gate");

        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        closeShooterGate();
    }

    /**
     * Sets explicit transfer motor power.
     * @param power Power level (-1.0 to 1.0)
     */
    public void setTransferPower(double power) {
        if (transferMotor != null) {
            transferMotor.setPower(power);
        }
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
        if (shooter1Gate != null) {
            shooter1Gate.setPosition(position);
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
        return shooter1Gate != null ? shooter1Gate.getPosition() : 0.0;
    }
}
