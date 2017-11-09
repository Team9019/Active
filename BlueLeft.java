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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   -
 *   -
 *   -
 *   -
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 */

@Autonomous(name="Blue Left", group="Autonomous")

public class BlueLeft extends LinearOpMode {

    /* Declare OpMode members. */
    private  HardRobot        robot   = new HardRobot();
    private ElapsedTime     runtime = new ElapsedTime();

    private static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    private static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double     DRIVE_SPEED             = 0.6;
    private static final double     TURN_SPEED              = 0.5;
    private static final double     LIFT_SPEED              = 0.7;
    private static final double     JWL_DST                 = 3;
    private static boolean blueFound = false;
    private static boolean redFound = false;
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.leftClaw.setPosition(0.9);            // S4: Stop and close the claw.
        robot.rightClaw.setPosition(0.1);
        robot.bigAss.setPosition(0.0);
       // sleep(1000);
        // pause for servos to move

        //telemetry.addData("Path", "Complete");
        //telemetry.update();

        robot.Color.enableLed(false);
        robot.Color.enableLed(true);



        // Send telemetry message to indicate successful Encoder reset
       // telemetry.addData("Path0",  "Starting at %7d :%7d",
        //                  robot.leftFront.getCurrentPosition(),
         //                 robot.leftBack.getCurrentPosition(),
          //                robot.rightFront.getCurrentPosition(),
           //               robot.rightBack.getCurrentPosition());
       // telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
       // encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        //encoderDrive(DRIVE_SPEED, -14, -14, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


        // 1) Drop Arm
        robot.bigAss.setPosition(0.73);


        // 2) Sense Color
        runtime.reset();
        while ( opModeIsActive() && runtime.seconds() < 5)
        {
            telemetry.addData("SenseJewel", "Red:" + robot.Color.red());
            telemetry.addData("SenseJewel", "Blue:" + robot.Color.blue());
            telemetry.update();

            idle();
        }

        if (robot.Color.red() >= 1 && robot.Color.red() <= 100) {
            redFound = true;
        }
        if (robot.Color.blue() >= 1 && robot.Color.blue() <= 100) {
            blueFound = true;
        }


        // 3) Lift Block
        robot.leftClaw.setPosition(0.5);
        robot.rightClaw.setPosition(0.8);
        sleep(200);

        encoderLift(LIFT_SPEED, 3, 2.0);



        // 4) Hit Jewel
        if (redFound) {
                encoderDrive(DRIVE_SPEED, JWL_DST, JWL_DST, 2.0);



        }
        if (blueFound) {

               encoderDrive(DRIVE_SPEED,-JWL_DST,-JWL_DST, 2.0);


        }


        // 5) Retract Arm
        robot.bigAss.setPosition(0.0);
        sleep(500);


        // 6) Forward/Right/Forward
        robot.leftFront.getCurrentPosition();
        robot.leftBack.getCurrentPosition();
        robot.rightFront.getCurrentPosition();
        robot.rightBack.getCurrentPosition();
        if (redFound) {
            encoderDrive(DRIVE_SPEED, (JWL_DST + 24)*(-1) , (JWL_DST + 24)*(-1), 4.0);
        }
        else if (blueFound) {
            encoderDrive(DRIVE_SPEED, (-JWL_DST + 24)*(-1), (-JWL_DST + 24)*(-1), 4.0);
        }

        encoderSlide(DRIVE_SPEED, 12, "L" , 2.0);

        encoderDrive(TURN_SPEED, 6, -6, 2.0);

        encoderDrive(DRIVE_SPEED, 10, 10, 2.0);


        // 7) Place Block
        encoderLift(LIFT_SPEED, -3, 1.0);

        robot.leftClaw.setPosition(0.9);
        robot.rightClaw.setPosition(0.1);

        encoderDrive(DRIVE_SPEED, 2, 2, 1.0);

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    private void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

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

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy())) {

                // Display it for the driver.
               // telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
               // telemetry.addData("Path2",  "Running at %7d :%7d",
               //                             robot.leftFront.getCurrentPosition(),
               //                             robot.leftBack.getCurrentPosition(),
               //                             robot.rightFront.getCurrentPosition(),
               //                             robot.rightBack.getCurrentPosition());
               // telemetry.update();
                idle();
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

            //  sleep(250);   // optional pause after each move
        }
    }
    private void encoderLift(double speed,
                              double liftInches,
                              double timeoutS) {
        int newLiftTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftTarget = robot.leftFront.getCurrentPosition() + robot.leftBack.getCurrentPosition() + (int)(liftInches * COUNTS_PER_INCH);

            robot.liftMotor.setTargetPosition(newLiftTarget);


            // Turn On RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftMotor.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftMotor.isBusy() )) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newLiftTarget);
                telemetry.addData("Path2",  "Running at %7d", robot.liftMotor.getCurrentPosition());

                telemetry.update();
                idle();
            }


            // Stop all motion;
            robot.liftMotor.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
    private void encoderSlide(double speed,
                              double Inches, String Direction,
                              double timeoutS) {

        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive())
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

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFront.isBusy() && robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy()))
            {

                // Display it for the driver.
                // telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                // telemetry.addData("Path2",  "Running at %7d :%7d",
                //                             robot.leftFront.getCurrentPosition(),
                //                             robot.leftBack.getCurrentPosition(),
                //                             robot.rightFront.getCurrentPosition(),
                //                             robot.rightBack.getCurrentPosition());
                // telemetry.update();
                idle();
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

            //  sleep(250);   // optional pause after each move
        }
    }
}
