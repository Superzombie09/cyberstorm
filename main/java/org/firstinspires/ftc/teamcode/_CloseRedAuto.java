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
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "_CloseRedAuto", group = "Robot")
public class _CloseRedAuto extends LinearOpMode {

    private ColorSensor Color_Sensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;
    static final double COUNTS_PER_MOTOR_REV = ((((1 + (46f / 17f))) * (1 + (46f / 17f))) * 28);    // eg: 5203 Yellow Jacket Planetary 435 RPM
    static final double DRIVE_GEAR_REDUCTION = 1;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.5;
    private int colorNumber = 0;


    @Override
    public void runOpMode() {

        waitForStart();

        Color_Sensor = hardwareMap.get(ColorSensor.class, "Color Sensor");
        FrontLeft = hardwareMap.get(DcMotor.class, "front_left");
        FrontRight = hardwareMap.get(DcMotor.class, "front_right");
        BackLeft = hardwareMap.get(DcMotor.class, "back_left");
        BackRight = hardwareMap.get(DcMotor.class, "back_right");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);

        Move(-0.225, -0.2, -0.225, -0.2, 3250); // moves robot to pixel

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

        Color_Sensor.enableLed(true);

        for (int i = 0; i < 400; i++){
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
            }

            telemetry.update();

            Color_Sensor.enableLed(false);

            // Red marker Red avg range: 15-30
            // Red marker Blue avg range: 0-10
            // Red marker Green avg range: 5-15
            // Blue marker Red avg range: 0-10
            // Blue marker Blue avg range: 15-30
            // Blue marker Green avg range: 5-15

        }
    }

    public void Move(double fl,
                     double fr,
                     double bl,
                     double br,
                     int time) {

        FrontLeft.setPower(fl);
        BackLeft.setPower(bl);
        FrontRight.setPower(fr);
        BackRight.setPower(br);

        sleep(time);

        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
    }

}