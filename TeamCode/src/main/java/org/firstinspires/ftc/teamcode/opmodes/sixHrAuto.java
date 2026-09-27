package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.DriveTrainMechanism;

@Autonomous
public class sixHrAuto extends LinearOpMode {
    private DriveTrainMechanism drivetrain = new DriveTrainMechanism();

    @Override

    public void runOpMode() {
        drivetrain.init(hardwareMap);
        waitForStart();
        if (opModeIsActive()) {
            moveRL(-0.3, 1);
            drivetrain.setMotorPowers(0,0,0,0);
        }
    }
        public void moveFB (double power, int seconds){
        drivetrain.setMotorPowers(power, power, power, power);
        sleep(seconds*1000);
        // negative is back
    }
    public void moveRL (double power, int seconds) {
        drivetrain.setMotorPowers(power,-power,-power,power);
        sleep(seconds*1000);
        // negative is left
    }
    }
