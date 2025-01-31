package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name="GoLeft")
public class GoLeft extends LinearOpMode{
    public DcMotor  leftFrontDrive = null;  // Done
    public DcMotor  rightFrontDrive = null; // Done
    public DcMotor  leftBackDrive = null; // Done
    public DcMotor  rightBackDrive = null;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lfd");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rfd");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lbd");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rbd");

        waitForStart();

        while (opModeIsActive()) {
            left(0.5, 5000);
        }

    }
    public void left(double power, long time) {
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);

    }
}
