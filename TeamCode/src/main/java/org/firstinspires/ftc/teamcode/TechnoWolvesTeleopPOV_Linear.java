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
public class TechnoWolvesTeleopPOV_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive = null;  // Done
    public DcMotor  rightFrontDrive = null; // Done
    public DcMotor  leftBackDrive = null; // Done
    public DcMotor  rightBackDrive = null; // Done
    public DcMotor RaisingMotor = null;
    public DcMotor ExtendingMainMotor = null; // Done
    public Servo RotatingServo = null; // Done
    public Servo    mainClaw = null; // Done
    public Servo PushingServo = null;



    double clawOffset = 0;
    double position = 0;
    double position2 = 0;



    public static final double MID_SERVO   =  0.5 ;

    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_DOWN_POWER    =  2 ;
    public static final double ARM_UP_POWER  = -2 ;
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
        RaisingMotor = hardwareMap.get(DcMotor.class, "rmm");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        ExtendingMainMotor.setDirection(DcMotor.Direction.FORWARD);


        RaisingMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Define and initialize ALL installed servos.
        mainClaw = hardwareMap.get(Servo.class, "mc");
        mainClaw.setPosition(0);

        PushingServo = hardwareMap.get(Servo.class, "ps");
        PushingServo.setPosition(0);

        RotatingServo = hardwareMap.get(Servo.class, "rs");
        RotatingServo.setPosition(0);

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
            if (left > 0.3)
                left = 0.3;

            if (right > 0.3)
                right = 0.3;

            if (position > 0.9)
                position = 0.9;
            if (position < 0)
                position = 0;

            if (position2 > 0.7)
                position2 = 0.7;
            if (position2 < 0)
                position2 = 0;

            // Output the safe vales to the motor drives.
            leftFrontDrive.setPower(left);
            leftBackDrive.setPower(left);
            rightBackDrive.setPower(right);
            rightFrontDrive.setPower(right);

            if (gamepad1.dpad_right) {
                leftFrontDrive.setPower(0.3);
                rightFrontDrive.setPower(-0.3);
                leftBackDrive.setPower(-0.3);
                rightBackDrive.setPower(0.3);
            }
            else if (gamepad1.dpad_left) {
                leftFrontDrive.setPower(-0.3);
                rightFrontDrive.setPower(0.3);
                leftBackDrive.setPower(0.3);
                rightBackDrive.setPower(-0.3);
            }
//            else {
//                leftFrontDrive.setPower(0);
//                rightFrontDrive.setPower(0);
//                leftBackDrive.setPower(0);
//                rightBackDrive.setPower(0);
//            }


            if (gamepad2.x) {
                position2 += 0.1;
                RotatingServo.setPosition(position2);
            }
            else if (gamepad2.b) {
                position2 -= 0.1;
                RotatingServo.setPosition(position2);
            }




            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            mainClaw.setPosition(MID_SERVO + clawOffset);

            if (gamepad2.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad2.left_bumper)
                clawOffset -= CLAW_SPEED;



            // Use gamepad buttons to move arm up (Y) and down (A)
            if (gamepad2.dpad_up)
                ExtendingMainMotor.setPower(ARM_UP_POWER);
            else if (gamepad2.dpad_down)
                ExtendingMainMotor.setPower(ARM_DOWN_POWER);
            else
                ExtendingMainMotor.setPower(0.0);

            if (gamepad1.dpad_up) {
                RaisingMotor.setPower(RAISE);
//                CounterWeightMotor.setPower(RAISE);
            }
            else if (gamepad1.dpad_down) {
                RaisingMotor.setPower(LOWER);
//                CounterWeightMotor.setPower(LOWER);
            }
            else {
                RaisingMotor.setPower(0.0);
            }

            if (gamepad2.y) {
                position += 0.1;
                PushingServo.setPosition(position);
            }
            else if (gamepad2.a) {
                position -= 0.1;
                PushingServo.setPosition(position);
            }




            // Send telemetry message to signify robot running;
            telemetry.addData("Left Front", leftFrontDrive.getPower());
            telemetry.addData("Left Back", leftBackDrive.getPower());
            telemetry.addData("Right Front", rightFrontDrive.getPower());
            telemetry.addData("Right Back", rightBackDrive.getPower());
            telemetry.addData("Claw", mainClaw.getPosition());
            telemetry.addData("Position", position);
            telemetry.addData("Position2", position2);



            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}
