package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Robot Controller1", group="Robot")
public class RobotController1 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFrontdrive = null;
    public DcMotor rightFrontdrive = null;
    public DcMotor leftBackdrive = null;
    public DcMotor rightBackdrive = null;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;

// Define and Initialize Motors
        leftFrontdrive = hardwareMap.get(DcMotor.class, "lfd");
        rightFrontdrive = hardwareMap.get(DcMotor.class, "rfd");
        leftBackdrive = hardwareMap.get(DcMotor.class, "lbd");
        rightBackdrive = hardwareMap.get(DcMotor.class, "rbd");

// Set motor directions (adjust if your robot drives backwards!)
        leftFrontdrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontdrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackdrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackdrive.setDirection(DcMotor.Direction.FORWARD);

// Set zero power behavior to BRAKE (stops faster)
        leftFrontdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Robot Ready. Press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

// Forward/back = left stick Y, turn = right stick X
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;

// Combine drive and turn
            left = drive + turn;
            right = drive - turn;

// --- NORMALIZATION ---
            double max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0) {
                left /= max;
                right /= max;
            }

// --- SLOW MODE ---
// Hold right bumper for half speed
            double speedFactor = gamepad1.right_bumper ? 0.5 : 1.0;
            left *= speedFactor;
            right *= speedFactor;

// Set motor power
            leftFrontdrive.setPower(left);
            leftBackdrive.setPower(left);
            rightFrontdrive.setPower(right);
            rightBackdrive.setPower(right);



// Telemetry for debugging
            telemetry.addData("Left Power", "%.2f", left);
            telemetry.addData("Right Power", "%.2f", right);
            telemetry.update();

            sleep(50); // keeps loop from running too fast
        }
    }
}