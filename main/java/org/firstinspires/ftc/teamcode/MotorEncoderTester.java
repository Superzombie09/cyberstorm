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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@Autonomous(name = "Motor Encoder Tester", group = "Robot")
public class MotorEncoderTester extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    //private DcMotor arm_rotationleft = null;
    private DcMotor arm_rotationright = null;
    //private DcMotor Extension = null;
    private Servo clawleft = null;
    private Servo clawright = null;
    private Boolean Clawstate = true;
    //private Integer Spike_Mark = 0;
   // private ColorSensor Color_Sensor = null;
    private ElapsedTime runtime = new ElapsedTime();


    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = ((((1 + (46f / 17f))) * (1 + (46f / 17f))) * 28);    // eg: 5203 Yellow Jacket Planetary 435 RPM
    static final double DRIVE_GEAR_REDUCTION = 1;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        //arm_rotationleft = hardwareMap.get(DcMotor.class, "Arm_Rotationleft");
        arm_rotationright = hardwareMap.get(DcMotor.class, "Arm_Rotationright");
        //Extension = hardwareMap.get(DcMotor.class, "Arm_Extension");
        clawleft = hardwareMap.get(Servo.class, "Claw-Left");
        clawright = hardwareMap.get(Servo.class, "Claw-Right");
//        //collection = hardwareMap.get(DcMotor.class, "CS");
        //Color_Sensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //arm_rotationleft.setDirection(DcMotor.Direction.FORWARD);
        arm_rotationright.setDirection(DcMotor.Direction.FORWARD);
        //Extension.setDirection(DcMotor.Direction.FORWARD);
        clawleft.setDirection(Servo.Direction.FORWARD);
        clawright.setDirection(Servo.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        clawleft.scaleRange(0.33, 0.88);
//        clawright.scaleRange(0.33, 0.88);


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftFrontDrive.getCurrentPosition(),
                leftBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrentPosition(),
                rightFrontDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        AutnomuousMethod(DRIVE_SPEED, 10, 10,
                0, 0, false,
                0, 0);
        AutnomuousMethod(DRIVE_SPEED, 0, 10,
                0, 0, false,
                0, 0);
        AutnomuousMethod(DRIVE_SPEED, 0, 0,
                10, 0, false,
                0, 0);
        AutnomuousMethod(DRIVE_SPEED, 0, 0,
                0, 10, false,
                0, 0);
        AutnomuousMethod(DRIVE_SPEED, 10, 10,
                10, 10, false,
                0, 0);
        AutnomuousMethod(DRIVE_SPEED, -10, 0,
                0, 0, false,
                0, 0);
        AutnomuousMethod(DRIVE_SPEED, 0, -10,
                0, 0, false,
                0, 0);
        AutnomuousMethod(DRIVE_SPEED, 0, 0,
                -10, 0, false,
                0, 0);
        AutnomuousMethod(DRIVE_SPEED, 0, 0,
                0, -10, false,
                0, 0);
        //add if statement for color sensor
        //if (Color_Sensor.green() == 0) {
        //AutnomuousMethod(0, 0, 0,
        //        0, 0, false,
        //        0, 0, 0);
        //}
        //AutnomuousMethod(TURN_SPEED, 10, 10,
        //        -10, -10, false,
        //        0, 0, 10);
        //if (Color_Sensor.green() == 0) {
       //     AutnomuousMethod(0, 0, 0,
        //            0, 0, false,
        //            0, 0, 0);
        //}
       // AutnomuousMethod(DRIVE_SPEED, 20, 20,
       //         20, 20, false,
        //        0, 0, 10);
        // strafe right or left depending on color sensor input(if statement)
        //place pixel and go to parking location(if statements)
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    public void AutnomuousMethod(double speed,
                                 double leftFrontInches, double leftBackInches, double rightFrontInches, double rightBackInches,
                                 boolean ClawChange, int ArmRotation, int ArmExtension_Compression) {
        int LeftFrontTarget = 0;
        int LeftBackTarget = 0;
        int RightFrontTarget = 0;
        int RightBackTarget = 0;

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            LeftFrontTarget = (int) (leftFrontInches * COUNTS_PER_INCH);
            LeftBackTarget = (int) (leftBackInches * COUNTS_PER_INCH) * 2;
            RightFrontTarget = (int) (rightFrontInches * COUNTS_PER_INCH);
            RightBackTarget = (int) (rightBackInches * COUNTS_PER_INCH) * 2;

            leftFrontDrive.setTargetPosition(LeftFrontTarget);
            leftBackDrive.setTargetPosition(LeftBackTarget);
            rightFrontDrive.setTargetPosition(RightFrontTarget);
            rightBackDrive.setTargetPosition(RightBackTarget);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData( "Target Position", leftFrontDrive.getTargetPosition());
            telemetry.addData( "Target Position", leftBackDrive.getTargetPosition());
            telemetry.addData( "Target Position", rightFrontDrive.getTargetPosition());
            telemetry.addData( "Target Position", rightBackDrive.getTargetPosition());
            telemetry.update();
            sleep(2000);

            //start motion.
            /*if (leftFrontDrive.getTargetPosition() !=  0) {
                if (LeftFrontTarget < 0) {
                    leftFrontDrive.setPower(-speed);
                } else {
                    leftFrontDrive.setPower(speed);
                }
            }
            if (leftBackDrive.getTargetPosition() != 0) {
                if (LeftBackTarget < 0) {
                    leftBackDrive.setPower(-speed);
                } else {
                    leftBackDrive.setPower(speed);
                }
            }
            if (rightFrontDrive.getTargetPosition() != 0) {
                if (RightFrontTarget < 0) {
                    rightFrontDrive.setPower(-speed);
                } else {
                    rightFrontDrive.setPower(speed);
                }
            }
            if (rightBackDrive.getTargetPosition() != 0) {
                if (RightBackTarget < 0) {
                    rightBackDrive.setPower(-speed);
                } else {
                    rightBackDrive.setPower(speed);
                }

            }
            */

            if (leftFrontDrive.getTargetPosition() !=  0) {
                    leftFrontDrive.setPower(speed);
            }
            if (leftBackDrive.getTargetPosition() != 0) {
                    leftBackDrive.setPower(speed);
            }
            if (rightFrontDrive.getTargetPosition() != 0) {
                    rightFrontDrive.setPower(speed);
            }
            if (rightBackDrive.getTargetPosition() != 0) {
                    rightBackDrive.setPower(speed);
                }

            while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
            // Display it for the driver.
            //telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newLeftBackTarget,   newRightFrontTarget,   newRightBackTarget);
            telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d",
                    leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.update();
            }

        // Stop all motion;
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(1000);   // optional pause after each move.
    }
}}