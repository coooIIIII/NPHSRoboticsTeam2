package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Autonomous1", group="Robot")
public class Autonomous1 extends LinearOpMode {

    // Drive motors
    private DcMotor leftFrontdrive, rightFrontdrive, leftBackdrive, rightBackdrive;
    private ElapsedTime runtime = new ElapsedTime();

    // Vision
    private VisionPortal myVisionPortal;
    private AprilTagProcessor myAprilTagProcessor;

    // Encoder constants
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

// Initialize drive motors
        leftFrontdrive = hardwareMap.get(DcMotor.class, "lfd");
        rightFrontdrive = hardwareMap.get(DcMotor.class, "rfd");
        leftBackdrive = hardwareMap.get(DcMotor.class, "lbd");
        rightBackdrive = hardwareMap.get(DcMotor.class, "rbd");

        leftFrontdrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontdrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackdrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackdrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

// Initialize AprilTag detection
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

// ----------------------------
// Step 4: Detect Obelisk AprilTag before match start
// ----------------------------
        int motifId = -1;
        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> currentDetections = myAprilTagProcessor.getDetections();
            if (!currentDetections.isEmpty()) {
                motifId = currentDetections.get(0).id;
                telemetry.addData("Pattern detected", motifId);
            } else {
                telemetry.addData("Pattern detected", "No tag yet");
            }
            telemetry.update();
        }

        waitForStart();

// Step 4: Select autonomous path based on motif ID
        switch (motifId) {
            case 21: runPatternA(); break;
            case 22: runPatternB(); break;
            case 23: runPatternC(); break;
            default: runDefaultPattern(); break;
        }
    }

    // ----------------------------
// Autonomous pattern placeholders
// ----------------------------
    public void runPatternA() {
        telemetry.addData("Auton", "Running Pattern A");
        telemetry.update();
// TODO: Add Limelight 3A logic for artifact collection & scoring
    }

    public void runPatternB() {
        telemetry.addData("Auton", "Running Pattern B");
        telemetry.update();
// TODO: Add Limelight 3A logic for artifact collection & scoring
    }

    public void runPatternC() {
        telemetry.addData("Auton", "Running Pattern C");
        telemetry.update();
// TODO: Add Limelight 3A logic for artifact collection & scoring
    }

    public void runDefaultPattern() {
        telemetry.addData("Auton", "Running Default Pattern");
        telemetry.update();
// Safe fallback path
    }

    // ----------------------------
// AprilTag setup
// ----------------------------
    public void initAprilTag() {
        myAprilTagProcessor = new AprilTagProcessor.Builder().build();

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(myAprilTagProcessor)
                .build();
    }

    // ----------------------------
// Optional: Encoder drive utility
// ----------------------------
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            newLeftTarget = leftFrontdrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontdrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            leftFrontdrive.setTargetPosition(newLeftTarget);
            rightFrontdrive.setTargetPosition(newRightTarget);
            leftBackdrive.setTargetPosition(newLeftTarget);
            rightBackdrive.setTargetPosition(newRightTarget);

            leftFrontdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftFrontdrive.setPower(Math.abs(speed));
            rightFrontdrive.setPower(Math.abs(speed));
            leftBackdrive.setPower(Math.abs(speed));
            rightBackdrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontdrive.isBusy() && rightFrontdrive.isBusy() &&
                            leftBackdrive.isBusy() && rightBackdrive.isBusy())) {

                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " %7d :%7d",
                        leftFrontdrive.getCurrentPosition(), rightFrontdrive.getCurrentPosition());
                telemetry.update();
            }

            leftFrontdrive.setPower(0);
            rightFrontdrive.setPower(0);
            leftBackdrive.setPower(0);
            rightBackdrive.setPower(0);

            leftFrontdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
}