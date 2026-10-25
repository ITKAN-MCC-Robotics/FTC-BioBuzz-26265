package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake Mechanism for Biobuzz robot.
 * Controls the goBILDA Yellowjacket motor "Intake".
 */
public class IntakeMechanism {
    public static final double DEFAULT_INTAKE_POWER = 1.0;

    private DcMotor intakeMotor;

    /**
     * Initializes hardware for intake motor.
     * @param hwMap HardwareMap from OpMode
     */
    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");

        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets explicit intake motor power.
     * @param power Power level (-1.0 to 1.0)
     */
    public void setIntakePower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    /**
     * Runs intake forward to collect pollen balls.
     */
    public void runIntake() {
        setIntakePower(DEFAULT_INTAKE_POWER);
    }

    /**
     * Reverses intake motor to eject pollen balls.
     */
    public void reverseIntake() {
        setIntakePower(-DEFAULT_INTAKE_POWER);
    }

    /**
     * Stops intake motor.
     */
    public void stopIntake() {
        setIntakePower(0.0);
    }

    /**
     * Returns current intake motor power.
     * @return motor power
     */
    public double getIntakePower() {
        return intakeMotor != null ? intakeMotor.getPower() : 0.0;
    }
}
