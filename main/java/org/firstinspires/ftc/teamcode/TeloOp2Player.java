/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele-Op 2 Player", group = "Linear OpMode")
public class TeloOp2Player extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private final DcMotor collection = null;
    private int Arm_State = 1;
    private int Clawstate = 1;

    private boolean AButtonWasPressed = false;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        DcMotor arm_rotationleft = hardwareMap.get(DcMotor.class, "Arm_Rotationleft");
        DcMotor arm_rotationright = hardwareMap.get(DcMotor.class, "Arm_Rotationright");
        //private DcMotor collection = null;
        DcMotor arm_extension = hardwareMap.get(DcMotor.class, "Arm_Extension");
        Servo clawleft = hardwareMap.get(Servo.class, "Claw-Left");
        Servo clawright = hardwareMap.get(Servo.class, "Claw-Right");
        //collection = hardwareMap.get(DcMotor.class, "CS");
        Servo airplane = hardwareMap.get(Servo.class, "Airplane");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        arm_rotationleft.setDirection(DcMotor.Direction.REVERSE);
        arm_rotationright.setDirection(DcMotor.Direction.FORWARD);
        arm_extension.setDirection((DcMotor.Direction.FORWARD));
        clawleft.setDirection(Servo.Direction.FORWARD);
        clawright.setDirection(Servo.Direction.FORWARD);
        airplane.setDirection(Servo.Direction.FORWARD);


        clawleft.scaleRange(0.05, 0.15);
        clawright.scaleRange(0.9, 1);
        airplane.scaleRange(0, 0.33);

        arm_rotationright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_rotationleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm_extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_rotationleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_rotationright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_rotationleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_rotationright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double Claw_Close = 0;
        double Claw_Open = 1;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max = 1;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double Movement = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double Strafe = gamepad1.left_stick_x;
            double Turn = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = Movement + 1.66*Strafe + Turn;
            double rightFrontPower = Movement - 1.66*Strafe - Turn;
            double leftBackPower = Movement - Strafe + Turn; // multiply this by a bit
            double rightBackPower = Movement + Strafe - Turn;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(max, Math.abs(leftFrontPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            leftFrontDrive.setPower(leftFrontPower / 2);
            rightFrontDrive.setPower(rightFrontPower / 2);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if (gamepad2.x) {
                telemetry.addData("Status", "left_bumper" + runtime.toString());

                if (Clawstate == 1) {
                    clawleft.setPosition(Claw_Open);
                    clawright.setPosition(Claw_Close);
                    Clawstate = 2;
                    sleep(500);
                } else {
                    clawleft.setPosition(Claw_Close);
                    clawright.setPosition(Claw_Open);
                    Clawstate = 1;
                    sleep(500);
                }
            }

            // if button is pressed arm will extend


            if (gamepad2.left_bumper) {
                arm_extension.setTargetPosition(0);
                arm_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_extension.setPower(0.5);
            } else if (gamepad2.right_bumper) {
                arm_extension.setTargetPosition(2500);
                arm_extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_extension.setPower(1);
            }
            // if button is pressed the arm base will go up
            if (gamepad2.y) {
                arm_rotationright.setTargetPosition(600);
                arm_rotationright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_rotationright.setPower(0.5);
                //arm_rotationright.setPower(0.3);
            } else if (gamepad2.a) {
                AButtonWasPressed = true;
                arm_rotationright.setTargetPosition(-10);
                arm_rotationright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_rotationright.setPower(0.3);
                sleep(250);
                //arm_rotationright.setPower(-0.25);
            } else if (gamepad2.b) {
                arm_rotationright.setTargetPosition(1000);
                arm_rotationright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_rotationright.setPower(0.5);
            }
            if (arm_rotationright.isBusy()) {
                arm_rotationleft.setPower(0);
                arm_rotationleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }   else {
                arm_rotationleft.setPower(0);
                arm_rotationleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (gamepad2.dpad_down) {
                airplane.setPosition(1);
            }
            if (gamepad2.dpad_right) {
                arm_rotationright.setTargetPosition(-10);
                arm_rotationright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_rotationright.setPower(0.5);
                sleep(250);
                arm_rotationright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm_rotationright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad2.dpad_left) {
                arm_rotationright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm_rotationright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                arm_rotationright.setTargetPosition(10);
                arm_rotationright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm_rotationright.setPower(0.5);
            }
            if (AButtonWasPressed) {
                if (!arm_rotationright.isBusy()) {
                    arm_rotationright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm_rotationright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    arm_rotationright.setTargetPosition(15);
                    arm_rotationright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm_rotationright.setPower(0.5);
                    AButtonWasPressed = false;
                }
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontDrive.getPower(), rightFrontDrive.getPower());
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
            telemetry.addData("arm rotationleft CP", arm_rotationleft.getCurrentPosition());
            telemetry.addData("arm rotationright CP", arm_rotationright.getCurrentPosition());
            telemetry.addData("arm extension CP", arm_extension.getCurrentPosition());
            telemetry.addData("arm rotationleft TP", arm_rotationleft.getTargetPosition());
            telemetry.addData("arm rotationright TP", arm_rotationright.getTargetPosition());
            telemetry.addData("arm extension TP", arm_extension.getTargetPosition());
            telemetry.addData("Arm_State", Arm_State);
            telemetry.addData("Claw", Clawstate);
            telemetry.update();
        }
    }
}