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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "_CloseBlueAuto", group = "Robot")
public class _CloseBlueAuto extends LinearOpMode {

    private ColorSensor Color_Sensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;
    private Servo pixelServo = null;
    static final double COUNTS_PER_MOTOR_REV = ((((1 + (46f / 17f))) * (1 + (46f / 17f))) * 28);    // eg: 5203 Yellow Jacket Planetary 435 RPM
    static final double DRIVE_GEAR_REDUCTION = 1;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;
    private int colorNumber = 0;
    private int dropPixel = 0;
    private int park = 0;

    public void colorCheck() {
        if (opModeIsActive()) {
            for (int i = 0; i < 120; i++) {
                telemetry.addData("Red", Color_Sensor.red());
                telemetry.addData("Green", Color_Sensor.green());
                telemetry.addData("Blue", Color_Sensor.blue());

                if (Color_Sensor.red() > 200) {
                    telemetry.addData("Color", "TrueRed");
                    colorNumber = 1; // 1 -> red
                }
                if (Color_Sensor.blue() > 200) {
                    telemetry.addData("Color", "TrueBlue");
                    colorNumber = 2; // 2 -> blue
                }
                telemetry.update();
            }
        }
    }

    public void Move(double fl,
                     double fr,
                     double bl,
                     double br,
                     int time) {
        if (opModeIsActive()) {
        FrontLeft.setPower(fl/2);
        BackLeft.setPower(bl);
        FrontRight.setPower(fr/2);
        BackRight.setPower(br);

        sleep(time);

        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }}
    public void PlacePixel() {
        if (opModeIsActive()) {
        if (colorNumber == 1 || colorNumber == 2) {
            telemetry.addData("drop pixel", "true");
            dropPixel = 1;
            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.FORWARD);
            FrontRight.setDirection(DcMotor.Direction.REVERSE);
            BackRight.setDirection(DcMotor.Direction.REVERSE);
            pixelServo.setPosition(1);
        }
        park += 1;
    }}
    public void BetterMove(double speed,
                                 double leftFrontInches,
                                 double leftBackInches,
                                 double rightFrontInches,
                                 double rightBackInches,
                                 Boolean Turn) {
//        int LeftFrontTarget = 0;
//        int LeftBackTarget = 0;
        int RightFrontTarget = 0;
        int RightBackTarget = 0;

        //leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            //LeftFrontTarget = (int) (leftFrontInches * COUNTS_PER_INCH);
            //LeftBackTarget = (int) (leftBackInches * COUNTS_PER_INCH) * 2;
            RightFrontTarget = (int) (rightFrontInches * COUNTS_PER_INCH);
            RightBackTarget = (int) (rightBackInches * COUNTS_PER_INCH) * 2;

            //leftFrontDrive.setTargetPosition(LeftFrontTarget);
            //leftBackDrive.setTargetPosition(LeftBackTarget);
            FrontRight.setTargetPosition(RightFrontTarget);
            BackRight.setTargetPosition(RightBackTarget);

            //leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //telemetry.addData( "Target Position", leftFrontDrive.getTargetPosition());
            //telemetry.addData( "Target Position", leftBackDrive.getTargetPosition());
            telemetry.addData("Target Position", FrontRight.getTargetPosition());
            telemetry.addData("Target Position", BackRight.getTargetPosition());
            telemetry.update();
            sleep(500);

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

            if (Turn) {

                if (FrontRight.getTargetPosition() != 0) {
                    FrontRight.setPower(speed/2);
                }

                if (BackRight.getTargetPosition() != 0) {
                    BackRight.setPower(speed);
                }

                if (leftFrontInches != 0 || leftBackInches != 0) {
                    while (FrontRight.getCurrentPosition() < FrontRight.getTargetPosition()
                            || BackRight.getCurrentPosition() < BackRight.getTargetPosition()) {
                        FrontLeft.setPower(1 * (-speed/2));
                        BackLeft.setPower(0.875 * -speed);
                    }
                    FrontLeft.setPower(0);
                    BackLeft.setPower(0);
                }

            } else {

                if (FrontRight.getTargetPosition() != 0) {
                    FrontRight.setPower(speed/2);
                }

                if (BackRight.getTargetPosition() != 0) {
                    BackRight.setPower(speed);
                }

                if (leftFrontInches != 0 || leftBackInches != 0) {
                    while (FrontRight.getCurrentPosition() < FrontRight.getTargetPosition()
                            || BackRight.getCurrentPosition() < BackRight.getTargetPosition()) {
                        FrontLeft.setPower(0.9 * (speed/2));
                        BackLeft.setPower(0.9 * speed);
                    }
                    FrontLeft.setPower(0);
                    BackLeft.setPower(0);
                }
            }
            while (FrontRight.isBusy()) {
                telemetry.update();
                //            while (leftFrontDrive.isBusy() || leftBackDrive.isBusy() || rightFrontDrive.isBusy() || rightBackDrive.isBusy()) {
            }
//                // Display it for the driver.
//            //telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newLeftBackTarget,   newRightFrontTarget,   newRightBackTarget);
//            telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d",
            //                   rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
//            telemetry.update();
//            }

            // Stop all motion;
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            //leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);   // optional pause after each move.
        }
    }
    public void Forward() {
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void Reverse() {
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {

        Color_Sensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        FrontLeft = hardwareMap.get(DcMotor.class, "front_left");
        FrontRight = hardwareMap.get(DcMotor.class, "front_right");
        BackLeft = hardwareMap.get(DcMotor.class, "back_left");
        BackRight = hardwareMap.get(DcMotor.class, "back_right");
        pixelServo = hardwareMap.get(Servo.class, "servo_pixel"); // don't forget to import servo

        pixelServo.setDirection(Servo.Direction.FORWARD);

        pixelServo.scaleRange(0, 0.7);

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();

        BetterMove(0.4, 30,30,30,30,false);
        // moves robot to check if thing is there
        colorCheck();
        if (colorNumber == 1 || colorNumber == 2) {
            Reverse();
            Move(0.15,0.125,0.15,0.125,300);

        }
        PlacePixel();
        if (dropPixel == 1 && park == 1) {
            sleep(1000);// if thing is there
/*            Reverse();
            Move(0.5,-0.5,-0.3,0.3, 300);
            BetterMove(0.5,27,27,27,27, false);
            Forward();
            BetterMove(0.15,3,3,3,3,false);
            Reverse();
            BetterMove(0.4, 20.5, 20.5,20.5, 20.5, true);
            BetterMove(0.8,45,45,45,45, false);
            BetterMove(0.4,20.5,20.5,20.5,20.5,true);
            BetterMove(0.8,50,50,50,50,false);
            Forward();
            BetterMove(0.4,20.5,20.5,20.5,20.5,true);
            BetterMove(0.8,25,25,25,25, false);
*///            BetterMove(0.5, )
//            BetterMove(0.4,62,62,62,62,true);
//            BetterMove(0.3,30,30,30,30,false);
//            BetterMove(0.3,48,48,48,48,true);
//            BetterMove(0.3,30,30,30,30,false);
//            BetterMove(0.3,18,18,18,18,true);
//            BetterMove(0.3,60,60,60,60,false);

        } else { // if thing isn't there
            Reverse();
            BetterMove(0.4,12,12,12,12 ,false);
            // goes backwards
            Move(0.5,-0.5,-0.3,0.3,1300);
            // strafes
            Forward();
            BetterMove(0.3,8,8,8,8,false);
            // moves backwards
            colorCheck();
            if (colorNumber == 1 || colorNumber == 2) {
                Move(-0.3,-0.3,-0.3,-0.3,500);
                Move(0.5,-0.5,-0.3,0.3, 400);
            }
            PlacePixel();
            sleep(2000);
            Forward();
            if (dropPixel == 1 && park == 2) { // parks
                sleep(1000);
//                Move(0.3,-0.3,-0.3,0.3,250);
//                BetterMove(0.4, 30, 30, 30, 30, false);
//                BetterMove(0.4, 48, 48, 48, 48, true);
//                BetterMove(0.4, 30, 30, 30, 30, false);
//                BetterMove(0.4, 18, 18, 18, 18, true);
//                BetterMove(0.4, 60, 60, 60, 60, false);
            } else { // drops pixel far left
                Reverse();
                BetterMove(0.4,2,2,2,2,false);
                Forward();
                BetterMove(0.4, 18, 19, 19, 19, true);
                BetterMove(0.4,11.5,11.5,11.5,11.5,false);
                pixelServo.setPosition(1);
                sleep(500);
                Forward();
/*                BetterMove(0.3, 30, 30, 30, 30, false);
                BetterMove(0.3, 48, 48, 48, 48, true);
                BetterMove(0.3, 30, 30, 30, 30, false);
                BetterMove(0.3, 18, 18, 18, 18, true);
                BetterMove(0.3, 60, 60, 60, 60, false);
*///
            }
        }





//        FrontLeft.setPower(-0.225);
//        BackLeft.setPower(-0.225);
//        FrontRight.setPower(-0.2);
//        BackRight.setPower(-0.2);
//
//        sleep(3250);
//
//        FrontLeft.setPower(0);
//        BackLeft.setPower(0);
//        FrontRight.setPower(0);
//        BackRight.setPower(0);

/*        Color_Sensor.enableLed(true);

        colorCheck();


        for (int i = 0; i < 100; i++) {
            telemetry.addData("Red", Color_Sensor.red());
            telemetry.addData("Green", Color_Sensor.green());
            telemetry.addData("Blue", Color_Sensor.blue());

            if (Color_Sensor.red() > Color_Sensor.blue() && Color_Sensor.red() > Color_Sensor.green()) {
                telemetry.addData("Color", "TrueRed");
               colorNumber = 1; // 1 -> red
            }

            if (Color_Sensor.blue() > Color_Sensor.red() && Color_Sensor.blue() > Color_Sensor.green()) {
                telemetry.addData("Color", "TrueBlue");
                colorNumber = 2; // 2 -> blue

            telemetry.update();
            }
        }


        Color_Sensor.enableLed(false);

        if (colorNumber == 2) {
            telemetry.addData("drop pixel", "true");
            dropPixel = 1;
            pixelServo.setPosition(1);
            park = 1;
        } else {
            ;
            Move(0.325, -0.3, 0.325, -0.3, 2000);
            Color_Sensor.enableLed(true);
            if (Color_Sensor.blue() > Color_Sensor.red() && Color_Sensor.blue() > Color_Sensor.green()) {
                telemetry.addData("Color", "TrueBlue");
                colorNumber = 2; // 2 -> blue
                if (colorNumber == 2) {
                    dropPixel = 1;
                    pixelServo.setPosition(0.5);
                    park = 2;
                } else {
                    Move(0.325, -0.3, 0.325, -0.3, 4000);
                    dropPixel = 1;
                    pixelServo.setPosition(0.5);
                    park = 3;
                }
            }
        }

*/
            // Red marker Red avg range: 15-30
            // Red marker Blue avg range: 0-10
            // Red marker Green avg range: 5-15
            // Blue marker Red avg range: 0-10
            // Blue marker Blue avg range: 15-30
            // Blue marker Green avg range: 5-15

    }
}