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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class Commands
{
    //private  HardRobot        robot   = new HardRobot();
    private static final double     COUNTS_PER_MOTOR_REV    = 1140 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = 0.4;
    public static final double     TURN_SPEED              = 0.5;
    public static final double     LIFT_SPEED              = 0.7;
    public static final double     JWL_DST                 = 2;
    public static boolean blueFound = false;
    public static boolean redFound = false;





    private ElapsedTime runtime  = new ElapsedTime();

    LinearOpMode opMode;
    private HardRobot robot;

    /* Constructor */
    public Commands(LinearOpMode opMode, HardRobot nrobot)
{
this.opMode = opMode;
this.robot = nrobot;
}
    public void encoderDrive(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBack.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));


            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy())) {


                opMode.idle();
            }


            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }

    public void encoderLift(double speed,
                             double liftInches,
                             double timeoutS) {
        int newLiftTarget;


        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftTarget = robot.liftMotor.getCurrentPosition() + (int)(liftInches * COUNTS_PER_INCH);

            robot.liftMotor.setTargetPosition(newLiftTarget);

            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotor.setPower(speed);

            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftMotor.isBusy() )) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1",  "Running to %7d", newLiftTarget);
                opMode.telemetry.addData("Path2",  "Running at %7d", robot.liftMotor.getCurrentPosition());

                opMode.telemetry.update();
                opMode.idle();
            }

            // Stop all motion;
            robot.liftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderSlide(double speed,
                              double Inches, String Direction,
                              double timeoutS) {

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive())
        {

            // Determine new target position, and pass to motor controller
            if (Direction.equals("L"))
            {
                newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
            }
            else // (Direction.equals("R"))
            {
                newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
                newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (-Inches * COUNTS_PER_INCH);
                newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            }
            robot.leftFront.setTargetPosition(newLeftFrontTarget);
            robot.leftBack.setTargetPosition(newLeftBackTarget);
            robot.rightFront.setTargetPosition(newRightFrontTarget);
            robot.rightBack.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFront.setPower(Math.abs(speed));
            robot.leftBack.setPower(Math.abs(speed));
            robot.rightFront.setPower(Math.abs(speed));
            robot.rightBack.setPower(Math.abs(speed));



            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()))
            {


                opMode.idle();
            }


            // Stop all motion;
            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void dropArm()
    {
        robot.colorServo.setPosition(0.73);
    }
    public void senseColor() {
        robot.Color.enableLed(true);
        runtime.reset();
        while ( opMode.opModeIsActive() && runtime.seconds() < 2)
        {
            opMode.telemetry.addData("SenseJewel", "Red:" + robot.Color.red());
            opMode.telemetry.addData("SenseJewel", "Blue:" + robot.Color.blue());
            opMode.telemetry.update();

            opMode.idle();
        }

        if (robot.Color.red() >= 1 && robot.Color.red() <= 100)
        {
            redFound = true;
        }
        if (robot.Color.blue() >= 1 && robot.Color.blue() <= 100)
        {
            blueFound = true;
        }
        robot.Color.enableLed(false);

    }
    public void liftBlock()
    {
        robot.leftClaw.setPosition(0.1);
        robot.rightClaw.setPosition(0.9);
        robot.lunderClaw.setPosition(0.4);
        robot.runderClaw.setPosition(0.3);
        opMode.sleep(200);

        encoderLift(LIFT_SPEED, 10, 2.0);

    }
    public void retractArm()
    {
        robot.colorServo.setPosition(0.1);
        opMode.sleep(500);
    }
    public void placeBlock()
    {
        encoderLift(LIFT_SPEED, -6, 1.0);

        robot.leftClaw.setPosition(0.9);
        robot.rightClaw.setPosition(0.1);
        robot.lunderClaw.setPosition(0.1);
        robot.runderClaw.setPosition(0.7);

        encoderDrive(DRIVE_SPEED, 4, 4, 1.0);
        encoderDrive(DRIVE_SPEED, -3,-3,1.0);
        encoderDrive(DRIVE_SPEED, 4, 4, 1.0);
        encoderDrive(DRIVE_SPEED, -3,-3,1.0);
    }
 }


