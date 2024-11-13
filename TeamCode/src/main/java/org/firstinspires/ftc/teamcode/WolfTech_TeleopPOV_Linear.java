/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Robot: Teleop POV", group="Robot")
public class WolfTech_TeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive = null;  // Done
    public DcMotor  rightFrontDrive = null; // Done
    public DcMotor  leftBackDrive = null; // Done
    public DcMotor  rightBackDrive = null; // Done
    public DcMotor RaisingMotor = null;
    public DcMotor ExtendingMainMotor = null; // Done
    public DcMotor RotatingMotor = null; // Done
    public Servo    mainClaw = null; // Done
    public Servo    shortClaw = null; // Done
    public Servo    tallClaw = null; // Done
    public DcMotor shortClawMotor = null; // Done
    public DcMotor tallClawMotor = null; // Done


    double clawOffset = 0;
    double clawOffset2 = 0;
    double clawOffset3 = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double MID_SERVO2 = 0.5;
    public static final double MID_SERVO3 = 0.5;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  2 ;
    public static final double ARM_DOWN_POWER  = -2 ;
    public static final double RAISE    =  -2 ;
    public static final double LOWER = 2 ;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
//        double max;

        // Define and Initialize Motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "lfd");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rfd");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lbd");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rbd");
        ExtendingMainMotor = hardwareMap.get(DcMotor.class, "emm");
        RotatingMotor = hardwareMap.get(DcMotor.class, "rm");
        shortClawMotor = hardwareMap.get(DcMotor.class, "scm");
        tallClawMotor = hardwareMap.get(DcMotor.class, "tcm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        ExtendingMainMotor.setDirection(DcMotor.Direction.FORWARD);
        RotatingMotor.setDirection(DcMotor.Direction.FORWARD);
        shortClawMotor.setDirection(DcMotor.Direction.FORWARD);
        tallClawMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RaisingMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Define and initialize ALL installed servos.
        mainClaw = hardwareMap.get(Servo.class, "mc");
        mainClaw.setPosition(MID_SERVO);

        shortClaw = hardwareMap.get(Servo.class, "sc");
        shortClaw.setPosition(MID_SERVO2);

        tallClaw = hardwareMap.get(Servo.class, "tc");
        tallClaw.setPosition(MID_SERVO3);
        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            if (left > 0.5)
                left = 0.5;

            if (right > 0.5)
                right = 0.5;

            // Output the safe vales to the motor drives.
            leftFrontDrive.setPower(left);
            leftBackDrive.setPower(left);
            rightBackDrive.setPower(right);
            rightFrontDrive.setPower(right);

            if (gamepad1.dpad_right) {
                leftFrontDrive.setPower(0.5);
                rightFrontDrive.setPower(-0.5);
                leftBackDrive.setPower(-0.5);
                rightBackDrive.setPower(0.5);
            }
            else if (gamepad1.dpad_left) {
                leftFrontDrive.setPower(-0.5);
                rightFrontDrive.setPower(0.5);
                leftBackDrive.setPower(0.5);
                rightBackDrive.setPower(-0.5);
            }
            else {
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
            }


            if (gamepad1.y)
                RotatingMotor.setPower(0.5);
            else if (gamepad1.a)
                RotatingMotor.setPower(-0.5);
            else
                RotatingMotor.setPower(0.0);


            if (gamepad1.y)
                RotatingMotor.setPower(0.5);
            else if (gamepad1.a)
                RotatingMotor.setPower(-0.5);
            else
                RotatingMotor.setPower(0.0);




            if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;

            if (gamepad2.y)
                clawOffset2 += CLAW_SPEED;
            else if (gamepad2.a)
                clawOffset2 -= CLAW_SPEED;

            if (gamepad2.x)
                clawOffset3 += CLAW_SPEED;
            else if (gamepad2.b)
                clawOffset3 += CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            mainClaw.setPosition(MID_SERVO + clawOffset);

            clawOffset2 = Range.clip(clawOffset2, -0.5, 0.5);
            shortClaw.setPosition(MID_SERVO2 + clawOffset2);

            clawOffset3 = Range.clip(clawOffset3, -0.5, 0.5);
            tallClaw.setPosition(MID_SERVO3 + clawOffset3);


            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad2.dpad_up)
                ExtendingMainMotor.setPower(ARM_UP_POWER);
            else if (gamepad2.dpad_down)
                ExtendingMainMotor.setPower(ARM_DOWN_POWER);
            else
                ExtendingMainMotor.setPower(0.0);

            if (gamepad1.dpad_up)
                RaisingMotor.setPower(RAISE);
            else if (gamepad1.dpad_down)
                ExtendingMainMotor.setPower(LOWER);
            else
                ExtendingMainMotor.setPower(0.0);


            if (gamepad2.right_bumper)
                shortClawMotor.setPower(0.5);
            else if (gamepad2.left_bumper)
                shortClawMotor.setPower(0.5);
            else
                shortClawMotor.setPower(0.0);

            if (gamepad2.left_stick_button)
                tallClawMotor.setPower(0.5);
            else if (gamepad2.right_stick_button)
                tallClawMotor.setPower(-0.5);
            else
                tallClawMotor.setPower(0.0);


            // Send telemetry message to signify robot running;
            telemetry.addData("Left Front", leftFrontDrive.getPower());
            telemetry.addData("Left Back", leftBackDrive.getPower());
            telemetry.addData("Right Front", rightFrontDrive.getPower());
            telemetry.addData("Right Back", rightBackDrive.getPower());


            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
